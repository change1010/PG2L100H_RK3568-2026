/*
 * Adapted from https://github.com/casiez/OneEuroFilter (C++ implementation).
 * Copyright 2019 Inria
 * Authors: Nicolas Roussel and Gery Casiez.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef VIDEO_DMA_DASHBOARD_ONE_EURO_H
#define VIDEO_DMA_DASHBOARD_ONE_EURO_H
#include <cmath>

// Value members replace the author's allocating LowPassFilter objects.
class DashboardOneEuro {
    double value = 0, derivative = 0;
    bool initialized = false;
    static double alpha(double hz, double dt) {
        return 1.0/(1.0+1.0/(6.283185307179586*hz*dt));
    }
public:
    void reset(double x) { value = x; derivative = 0; initialized = true; }
    double update(double x, double dt, double minimum, double beta) {
        if (!initialized || dt <= 0 || dt > .1) { reset(x); return x; }
        // Match the author's corrected implementation: previous FILTERED value.
        const double a = alpha(1,dt);
        derivative += a*((x-value)/dt-derivative);
        value += alpha(minimum+beta*std::abs(derivative),dt)*(x-value);
        return value;
    }
};
#endif
