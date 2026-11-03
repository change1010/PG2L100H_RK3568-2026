// Copyright 2023 TIER IV, Inc. Licensed under Apache-2.0.
// Modified CPU preprocessing/YOLOX adapter; attribution in
// ../models/traffic_light_specialist/NOTICE.md.
#include "light_specialist.h"
#include "rknn_api.h"
#include "video_dma_frame.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int SIDE = 416, ROWS = 3549, COLS = 8;
struct Box {
    float x0, y0, x1, y1, score;
    int cls, crop, anchor;
    bool edge;
    int refined = 0;
};
uint64_t clock_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
float area(const Box &b) { return std::max(0.f,b.x1-b.x0)*std::max(0.f,b.y1-b.y0); }
float overlap(const Box &a, const Box &b, bool coverage = false)
{
    const float inter = std::max(0.f,std::min(a.x1,b.x1)-std::max(a.x0,b.x0)) *
                        std::max(0.f,std::min(a.y1,b.y1)-std::max(a.y0,b.y0));
    const float denominator = coverage ? area(b) : area(a)+area(b)-inter;
    return denominator > 0 ? inter/denominator : 0;
}
std::vector<Box> nms(std::vector<Box> boxes, float threshold, bool class_aware)
{
    std::stable_sort(boxes.begin(),boxes.end(),[](const Box &a,const Box &b){return a.score>b.score;});
    std::vector<Box> kept;
    for (const auto &b : boxes) {
        bool duplicate = false;
        for (const auto &old : kept)
            if ((!class_aware || old.cls == b.cls) && overlap(old,b)>threshold) { duplicate=true; break; }
        if (!duplicate) kept.push_back(b);
    }
    return kept;
}
void dump(const char *prefix, int crop, const char *suffix, const void *data, size_t bytes)
{
    if (!prefix) return;
    const std::string path = std::string(prefix)+"_"+std::to_string(crop)+suffix;
    FILE *f = std::fopen(path.c_str(),"wb");
    if (!f) throw std::runtime_error("dump open");
    const bool ok = std::fwrite(data,1,bytes,f)==bytes;
    const int rc = std::fclose(f);
    if (!ok || rc) throw std::runtime_error("dump write");
}
std::vector<unsigned char> preprocess_legacy(const cv::Mat &image, const int roi[4], float &scale)
{
    const int w=roi[2]-roi[0], h=roi[3]-roi[1];
    // Match the archived CUDA equations and Python screening transcription.
    const double ratio = std::min(double(SIDE)/w,double(SIDE)/h);
    scale = float(ratio);
    const int rw=int(ratio*w), rh=int(ratio*h);
    std::vector<unsigned char> result(SIDE*SIDE*3,114);
    for (int y=0; y<rh; ++y) {
        const float cy=(float(y)+.5f)/scale, ry=std::floor(cy+.5f);
        const int iy=std::max(0,std::min(h-2,int(ry)-1));
        const double wy=double((1.f+ry-cy)/2.f);
        for (int x=0; x<rw; ++x) {
            const float cx=(float(x)+.5f)/scale, rx=std::floor(cx+.5f);
            const int ix=std::max(0,std::min(w-2,int(rx)-1));
            const double wx=double((1.f+rx-cx)/2.f);
            const auto a=image.at<cv::Vec3b>(roi[1]+iy,roi[0]+ix);
            const auto b=image.at<cv::Vec3b>(roi[1]+iy,roi[0]+ix+1);
            const auto c=image.at<cv::Vec3b>(roi[1]+iy+1,roi[0]+ix);
            const auto d=image.at<cv::Vec3b>(roi[1]+iy+1,roi[0]+ix+1);
            for (int ch=0; ch<3; ++ch) {
                const double value=(a[ch]*(1-wx)+b[ch]*wx)*(1-wy)+(c[ch]*(1-wx)+d[ch]*wx)*wy;
                result[(y*SIDE+x)*3+ch]=static_cast<unsigned char>(std::floor(value+.5));
            }
        }
    }
    return result;
}
std::vector<unsigned char> preprocess_mapped(const cv::Mat &image, const int roi[4], float &scale)
{
    const int w=roi[2]-roi[0], h=roi[3]-roi[1];
    const double ratio=std::min(double(SIDE)/w,double(SIDE)/h);
    scale=float(ratio);
    const int rw=int(ratio*w), rh=int(ratio*h);
    std::array<int,SIDE> offsets{};
    std::array<double,SIDE> weights{};
    for (int x=0;x<rw;++x) {
        const float cx=(float(x)+.5f)/scale, rx=std::floor(cx+.5f);
        offsets[x]=(roi[0]+std::max(0,std::min(w-2,int(rx)-1)))*3;
        weights[x]=double((1.f+rx-cx)/2.f);
    }
    std::vector<unsigned char> result(SIDE*SIDE*3,114);
    for (int y=0;y<rh;++y) {
        const float cy=(float(y)+.5f)/scale, ry=std::floor(cy+.5f);
        const int iy=roi[1]+std::max(0,std::min(h-2,int(ry)-1));
        const double wy=double((1.f+ry-cy)/2.f);
        const auto *top=image.ptr<unsigned char>(iy);
        const auto *bottom=image.ptr<unsigned char>(iy+1);
        auto *target=result.data()+y*SIDE*3;
        for (int x=0;x<rw;++x) {
            const int ix=offsets[x];
            const double wx=weights[x];
            for (int ch=0;ch<3;++ch) {
                const double value=
                    (top[ix+ch]*(1-wx)+top[ix+3+ch]*wx)*(1-wy)+
                    (bottom[ix+ch]*(1-wx)+bottom[ix+3+ch]*wx)*wy;
                // Interpolation is nonnegative; truncation equals floor here.
                target[x*3+ch]=static_cast<unsigned char>(value+.5);
            }
        }
    }
    return result;
}
class Engine {
    rknn_context ctx = 0;
    bool initialized = false;
    unsigned output_count = 0;
public:
    ~Engine() { if (ctx) rknn_destroy(ctx); }
    void open()
    {
        if (initialized) return;
        const char *path=std::getenv("VDMA_LIGHT_MODEL");
        if (!path || !*path) throw std::runtime_error("VDMA_LIGHT_MODEL missing");
        if (rknn_init(&ctx,const_cast<char *>(path),0,0,nullptr)) throw std::runtime_error("rknn_init");
        rknn_input_output_num io{};
        rknn_tensor_attr in{},out{};
        if (rknn_query(ctx,RKNN_QUERY_IN_OUT_NUM,&io,sizeof(io)) ||
            io.n_input!=1 || (io.n_output!=1 && io.n_output!=9) ||
            rknn_query(ctx,RKNN_QUERY_INPUT_ATTR,&in,sizeof(in)) ||
            in.n_dims!=4 || in.dims[0]!=1 || in.n_elems!=SIDE*SIDE*3 ||
            !((in.fmt==RKNN_TENSOR_NCHW && in.dims[1]==3 && in.dims[2]==SIDE && in.dims[3]==SIDE) ||
              (in.fmt==RKNN_TENSOR_NHWC && in.dims[1]==SIDE && in.dims[2]==SIDE && in.dims[3]==3))) {
            rknn_destroy(ctx); ctx=0;
            throw std::runtime_error("specialist tensor contract mismatch");
        }
        for (unsigned i=0;i<io.n_output;++i) {
            out={}; out.index=i;
            bool invalid=rknn_query(ctx,RKNN_QUERY_OUTPUT_ATTR,&out,sizeof(out))!=0;
            if (io.n_output==1)
                invalid=invalid || out.n_dims!=3 || out.dims[0]!=1 ||
                        out.dims[1]!=ROWS || out.dims[2]!=COLS || out.n_elems!=ROWS*COLS;
            else {
                const unsigned grid=SIDE/(8U<<(i/3));
                const unsigned channels=i%3==0 ? 4 : i%3==1 ? 1 : 3;
                invalid=invalid || out.n_dims!=4 || out.fmt!=RKNN_TENSOR_NCHW ||
                        out.dims[0]!=1 || out.dims[1]!=channels ||
                        out.dims[2]!=grid || out.dims[3]!=grid ||
                        out.n_elems!=channels*grid*grid;
            }
            if (invalid) {
                rknn_destroy(ctx); ctx=0;
                throw std::runtime_error("specialist output tensor contract mismatch");
            }
            std::fprintf(stderr,"LIGHT_SPECIALIST_TENSOR index=%u type=%d scale=%.9g zp=%d\n",
                         i,out.type,double(out.scale),out.zp);
        }
        output_count=io.n_output;
        initialized=true;
        std::fprintf(stderr,"LIGHT_SPECIALIST_MODEL input=416x416_BGR_scale1 output_count=%u input_type=%d output_type=%d\n",
                     output_count,in.type,out.type);
    }
    std::vector<float> run(std::vector<unsigned char> &pixels)
    {
        open();
        rknn_input in{};
        in.buf=pixels.data(); in.size=pixels.size(); in.type=RKNN_TENSOR_UINT8;
        in.fmt=RKNN_TENSOR_NHWC; in.pass_through=0;
        rknn_run_extend extend{}; extend.timeout_ms=2000; extend.fence_fd=-1;
        if (rknn_inputs_set(ctx,1,&in) || rknn_run(ctx,&extend)) throw std::runtime_error("NPU run");
        std::array<rknn_output,9> outputs{};
        for (unsigned i=0;i<output_count;++i) {
            outputs[i].index=i; outputs[i].want_float=1;
        }
        if (rknn_outputs_get(ctx,output_count,outputs.data(),nullptr))
            throw std::runtime_error("outputs_get");
        bool invalid=false;
        for (unsigned i=0;i<output_count;++i) {
            const unsigned grid=SIDE/(8U<<(i/3));
            const unsigned channels=i%3==0 ? 4 : i%3==1 ? 1 : 3;
            const unsigned elements=output_count==1 ? ROWS*COLS : grid*grid*channels;
            invalid=invalid || !outputs[i].buf || outputs[i].size!=elements*sizeof(float);
        }
        if (invalid) {
            rknn_outputs_release(ctx,output_count,outputs.data());
            throw std::runtime_error("output bytes mismatch");
        }
        std::vector<float> result(ROWS*COLS);
        if (output_count==1) {
            const auto *p=static_cast<const float *>(outputs[0].buf);
            std::copy(p,p+ROWS*COLS,result.begin());
        } else {
            unsigned anchor=0;
            for (unsigned level=0;level<3;++level) {
                const unsigned grid=SIDE/(8U<<level), count=grid*grid;
                const auto *boxes=static_cast<const float *>(outputs[level*3].buf);
                const auto *objects=static_cast<const float *>(outputs[level*3+1].buf);
                const auto *classes=static_cast<const float *>(outputs[level*3+2].buf);
                for (unsigned i=0;i<count;++i,++anchor) {
                    auto *row=result.data()+anchor*COLS;
                    for (unsigned ch=0;ch<4;++ch) row[ch]=boxes[ch*count+i];
                    row[4]=objects[i];
                    for (unsigned ch=0;ch<3;++ch) row[5+ch]=classes[ch*count+i];
                }
            }
        }
        const int rc=rknn_outputs_release(ctx,output_count,outputs.data());
        if (rc) throw std::runtime_error("outputs_release");
        return result;
    }
};
std::vector<Box> decode(const std::vector<float> &raw, const int roi[4], float scale, int crop)
{
    for (size_t i=0; i<raw.size(); ++i)
        if (!std::isfinite(raw[i]) || (i%8>=4 && (raw[i]<0 || raw[i]>1)))
            throw std::runtime_error("invalid output probability");
    std::vector<Box> boxes;
    int row=0;
    for (int stride : {8,16,32}) {
        const int grid=SIDE/stride;
        for (int y=0; y<grid; ++y) for (int x=0; x<grid; ++x,++row) {
            const float *a=&raw[row*COLS];
            const double cx=(double(a[0])+x)*stride,cy=(double(a[1])+y)*stride;
            // exp(float32), followed by float64 grid/stride arithmetic in Python.
            const double w=std::exp(std::max(-30.f,std::min(30.f,a[2])))*stride;
            const double h=std::exp(std::max(-30.f,std::min(30.f,a[3])))*stride;
            for (int cls=0; cls<3; ++cls) {
                const float score=a[4]*a[5+cls];
                if (score<=.3f) continue;
                Box b{float(std::max(double(roi[0]),std::min(double(roi[2]),(cx-w/2)/scale+roi[0]))),
                    float(std::max(double(roi[1]),std::min(double(roi[3]),(cy-h/2)/scale+roi[1]))),
                    float(std::max(double(roi[0]),std::min(double(roi[2]),(cx+w/2)/scale+roi[0]))),
                    float(std::max(double(roi[1]),std::min(double(roi[3]),(cy+h/2)/scale+roi[1]))),
                    score,cls,crop,row,false,0};
                b.edge=crop && (b.x0<=roi[0]+3 || b.y0<=roi[1]+3 || b.x1>=roi[2]-3 || b.y1>=roi[3]-3);
                if (area(b)>0) boxes.push_back(b);
                if (boxes.size()>1000) throw std::runtime_error("candidate overflow");
            }
        }
    }
    return nms(boxes,.65f,false);
}
std::vector<Box> refine(const cv::Mat &image, const std::vector<Box> &boxes)
{
    std::vector<Box> result;
    for (auto box : boxes) {
        const float w=box.x1-box.x0,h=box.y1-box.y0;
        unsigned matches=0;
        if (!box.crop) for (const auto &other : boxes) {
            const float bw=other.x1-other.x0,bh=other.y1-other.y0;
            if (other.crop && other.cls==box.cls && !other.edge && overlap(box,other)>=.15 &&
                .4<=w*h/(bw*bh) && w*h/(bw*bh)<=2.5 &&
                std::abs((box.x0+box.x1-other.x0-other.x1)/2)<=.7*std::max(w,bw) &&
                std::abs((box.y0+box.y1-other.y0-other.y1)/2)<=.5*std::max(h,bh)) matches++;
        }
        if (matches==1) continue;
        if (box.cls==1 && w/h>=2 && w/h<=6) {
            const int x0=std::max(0,int(std::floor(box.x0-w*.5))), y0=std::max(0,int(std::floor(box.y0-h)));
            const int x1=std::min(1920,int(std::ceil(box.x1+w*.5))), y1=std::min(1080,int(std::ceil(box.y1+h)));
            cv::Mat gray,edges;
            cv::cvtColor(image(cv::Rect(x0,y0,x1-x0,y1-y0)),gray,cv::COLOR_BGR2GRAY);
            cv::Canny(gray,edges,60,140);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(edges,contours,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
            std::vector<Box> options;
            for (const auto &contour : contours) {
                const cv::Rect r=cv::boundingRect(contour);
                Box b=box;
                b.x0=r.x+x0; b.y0=r.y+y0; b.x1=b.x0+r.width; b.y1=b.y0+r.height;
                const double ratio=double(r.width)/r.height,relative=area(b)/area(box);
                if (ratio>=2 && ratio<=5.5 && overlap(b,box)>=.5 &&
                    cv::contourArea(contour)/(r.width*r.height)>=.5 &&
                    relative>=.6 && relative<=2.2 && r.height/h>=.7 && r.height/h<=2) options.push_back(b);
            }
            std::stable_sort(options.begin(),options.end(),[&](const Box&a,const Box&b){return overlap(a,box)>overlap(b,box);});
            bool unambiguous=!options.empty();
            for (size_t i=1; i<options.size(); ++i) if (overlap(options[0],options[i])<.7) unambiguous=false;
            if (unambiguous) { box=options[0]; box.refined=1; }
        }
        result.push_back(box);
    }
    return result;
}
bool cancelled(const vdma_snapshot_input *input)
{
    return input->cancelled && input->cancelled(input->cancel_context);
}

struct Group {
    std::vector<Box> heads;
    bool valid = false;
};

Group form_group(const cv::Mat &image, std::vector<Box> candidates,
                 const int scope[4], unsigned layout)
{
    candidates.erase(std::remove_if(candidates.begin(),candidates.end(),
        [](const Box &b){return b.edge;}),candidates.end());
    candidates=refine(image,nms(candidates,.5f,true));
    const Box region{float(scope[0]),float(scope[1]),float(scope[2]),float(scope[3]),0,0,0,0,false,0};
    std::vector<Box> selected;
    for (const auto &b : candidates) {
        const float w=b.x1-b.x0,h=b.y1-b.y0;
        if (b.cls==1 && !b.edge && overlap(region,b,true)>=.98f &&
            w/h>=2 && w/h<=6 && w>=18 && h>=7) selected.push_back(b);
    }
    std::stable_sort(selected.begin(),selected.end(),
        [](const Box&a,const Box&b){return area(a)>area(b);});
    Group group;
    for (const auto &b : selected) {
        bool nested=false;
        for (const auto &old : group.heads) if (overlap(old,b,true)>=.85) nested=true;
        if (!nested) group.heads.push_back(b);
    }
    std::sort(group.heads.begin(),group.heads.end(),
        [](const Box&a,const Box&b){return a.x0<b.x0;});
    const bool two_head_fallback=layout==3 && group.heads.size()==2;
    bool valid=group.heads.size()==layout || two_head_fallback;
    float minw=1e9,maxw=0,minh=1e9,maxh=0,mincy=1e9,maxcy=0;
    for (size_t i=0; i<group.heads.size(); ++i) {
        const auto &b=group.heads[i];
        minw=std::min(minw,b.x1-b.x0); maxw=std::max(maxw,b.x1-b.x0);
        minh=std::min(minh,b.y1-b.y0); maxh=std::max(maxh,b.y1-b.y0);
        mincy=std::min(mincy,(b.y0+b.y1)/2); maxcy=std::max(maxcy,(b.y0+b.y1)/2);
        if (i && group.heads[i-1].x1>=b.x0) valid=false;
    }
    group.valid=valid && maxw<=minw*2 && maxh<=minh*2 && maxcy-mincy<=maxh;
    return group;
}
}

extern "C" const light_specialist_profile *light_specialist_profile_at(unsigned index)
{
    static const light_specialist_profile profiles[]={
        {{800,250,1600,700},3},{{800,250,1700,700},3},
        {{500,150,1400,650},2},{{650,300,1250,750},2}};
    return index<4 ? &profiles[index] : nullptr;
}

extern "C" int light_specialist_analyze(const vdma_snapshot_input *input, const int scope[4],
    unsigned layout, vdma_snapshot_output *output, detection_overlay *overlay,
    adas_signal_test_observation *observation, const char *dump_prefix)
{
    static Engine engine;
    const uint64_t started=clock_ns();
    std::memset(output,0,sizeof(*output)); std::memset(overlay,0,sizeof(*overlay));
    observation->groups_valid=0; observation->group_count=0; observation->group_signature=0;
    observation->reason=ADAS_TEST_FAILED;
    try {
        const char *policy=std::getenv("VDMA_LIGHT_SEARCH_POLICY");
        if (!policy || !*policy) policy="roi-first";
        if (std::strcmp(policy,"dual") && std::strcmp(policy,"roi-first"))
            throw std::runtime_error("invalid VDMA_LIGHT_SEARCH_POLICY");
        const char *preprocessor=std::getenv("VDMA_LIGHT_PREPROCESS");
        if (!preprocessor || !*preprocessor) preprocessor="mapped";
        if (std::strcmp(preprocessor,"legacy") && std::strcmp(preprocessor,"mapped"))
            throw std::runtime_error("invalid VDMA_LIGHT_PREPROCESS");
        const bool mapped=!std::strcmp(preprocessor,"mapped");
        const bool roi_first=!std::strcmp(policy,"roi-first");
        if (!input->pixels || input->header.width!=1920 || input->header.height!=1080 ||
            input->header.stride!=3840 || input->header.bytes!=1920*1080*2 ||
            !vdma_frame_valid(input->identity.slot,&input->header) ||
            input->identity.canvas_width!=1920 || input->identity.canvas_height!=1080 ||
            input->identity.source_id!=input->identity.layout || input->identity.layout>=4 ||
            input->header.sequence!=input->identity.frame_sequence || input->header.layout!=input->identity.layout ||
            !scope || scope[0]<0 || scope[1]<0 || scope[2]>1920 || scope[3]>1080 ||
            scope[2]-scope[0]<2 || scope[3]-scope[1]<2 || (layout!=2 && layout!=3))
            throw std::runtime_error("invalid frozen input/profile");
        cv::setNumThreads(1);
        const cv::Mat packed(1080,1920,CV_8UC2,const_cast<unsigned char *>(input->pixels));
        cv::Mat bgr; cv::cvtColor(packed,bgr,cv::COLOR_BGR5652BGR);
        const int full[4]={0,0,1920,1080};
        std::vector<Box> candidates;
        Group group;
        unsigned searches=0;
        double npu_ms=0,preprocess_ms=0;
        for (int index=0; index<2; ++index) {
            if (cancelled(input)) return VDMA_PROCESS_CANCELLED;
            const int crop=roi_first ? 1-index : index;
            const int *roi=crop ? scope : full;
            float scale=0;
            uint64_t begin=clock_ns();
            auto pixels=mapped ? preprocess_mapped(bgr,roi,scale) :
                                 preprocess_legacy(bgr,roi,scale);
            preprocess_ms+=(clock_ns()-begin)/1e6;
            dump(dump_prefix,crop,".input",pixels.data(),pixels.size());
            begin=clock_ns();
            auto raw=engine.run(pixels);
            npu_ms+=(clock_ns()-begin)/1e6;
            dump(dump_prefix,crop,".output",raw.data(),raw.size()*sizeof(float));
            const auto boxes=decode(raw,roi,scale,crop);
            candidates.insert(candidates.end(),boxes.begin(),boxes.end());
            ++searches;
            if (roi_first || index==1) group=form_group(bgr,candidates,scope,layout);
            // Prefer the declared three-head group before accepting two heads.
            if (roi_first && group.valid && group.heads.size()==layout) break;
        }
        if (cancelled(input)) return VDMA_PROCESS_CANCELLED;
        const auto &unique=group.heads;
        const bool valid=group.valid;
        observation->reason=valid ? ADAS_TEST_NONE : ADAS_TEST_GROUP_UNKNOWN;
        const unsigned accepted_count=valid ? unsigned(unique.size()) : 0;
        observation->groups_valid=valid; observation->group_count=accepted_count;
        observation->layout=accepted_count==3 ? ADAS_TEST_THREE_ARROW_CIRCLES : ADAS_TEST_TWO_CIRCLES;
        // IDs identify ordered housings in these immutable pixels, never a visual hold.
        uint64_t signature=UINT64_C(1469598103934665603);
        auto mix=[&](unsigned v){signature=(signature^v)*UINT64_C(1099511628211);};
        mix(layout); mix(accepted_count); for (int j=0;j<4;++j) mix(scope[j]);
        for (const auto &b : unique) {
            mix(unsigned(std::nearbyint(b.x0))); mix(unsigned(std::nearbyint(b.y0)));
            mix(unsigned(std::nearbyint(b.x1))); mix(unsigned(std::nearbyint(b.y1)));
        }
        observation->group_signature=valid ? signature : 0;
        const auto &id=input->identity;
        overlay->state=DETECTION_RUNNING; overlay->valid=1;
        overlay->stream_epoch=id.stream_epoch; overlay->source_id=id.source_id;
        overlay->layout=id.layout; overlay->config_generation=id.config_generation;
        overlay->frame_sequence=id.frame_sequence; overlay->observed_ns=input->observed_ns;
        overlay->valid_until_ns=UINT64_MAX;
        std::printf("LIGHT_SPECIALIST_RESULT {\"sequence\":%llu,\"frame\":%u,\"scope\":[%d,%d,%d,%d],\"expected\":%u,\"valid\":%s,\"observed\":%zu,\"accepted_count\":%u,\"two_head_fallback\":%s,\"policy\":\"%s\",\"searches\":%u,\"fallback\":%s,\"heads\":[",
            static_cast<unsigned long long>(observation->sequence),id.frame_sequence,scope[0],scope[1],scope[2],scope[3],
            layout,valid?"true":"false",unique.size(),accepted_count,
            valid && layout==3 && accepted_count==2 ? "true" : "false",policy,searches,
            roi_first && searches==2 ? "true" : "false");
        for (size_t i=0;i<unique.size();++i) {
            const auto &b=unique[i];
            int roi[4]={int(std::nearbyint(b.x0)),int(std::nearbyint(b.y0)),int(std::nearbyint(b.x1)),int(std::nearbyint(b.y1))};
            unsigned color=0,evidence=0;
            if (light_specialist_sample_color(input->pixels,roi,&color,&evidence)) throw std::runtime_error("color ROI");
            if (overlay->count<DETECTION_OVERLAY_CAPACITY) {
                auto &o=overlay->boxes[overlay->count++];
                o.x0=roi[0];o.y0=roi[1];o.x1=roi[2];o.y1=roi[3];
                o.model_class=9;o.confidence_q10000=unsigned(b.score*10000);o.signal=1;o.light_color=color;
            }
            if (valid) {
                auto &g=observation->groups[i];
                g.id=signature^(UINT64_C(0x9e3779b97f4a7c15)*(i+1));
                if (!g.id) g.id=i+1;
                g.identity_valid=1;g.color=color;g.color_valid=color>=1 && color<=3;
            }
            std::printf("%s{\"box\":[%d,%d,%d,%d],\"color\":%u,\"evidence\":%u,\"refined\":%d,\"score\":%.7g}",
                i?",":"",roi[0],roi[1],roi[2],roi[3],color,evidence,b.refined,double(b.score));
        }
        overlay->latency_ms=(clock_ns()-started)/1e6;
        std::printf("],\"preprocessor\":\"%s\",\"preprocess_ms\":%.3f,\"npu_ms\":%.3f,\"total_ms\":%.3f}\n",preprocessor,preprocess_ms,npu_ms,overlay->latency_ms);
        std::fflush(stdout);
        output->diagnostic_value=1;
        return VDMA_PROCESS_OK;
    } catch (const std::exception &e) {
        std::fprintf(stderr,"LIGHT_SPECIALIST_ERROR %s\n",e.what());
        std::memset(overlay,0,sizeof(*overlay)); overlay->state=DETECTION_ERROR;
        observation->reason=ADAS_TEST_FAILED; observation->groups_valid=0;
        output->error=EIO;
        return VDMA_PROCESS_FAILED;
    }
}
