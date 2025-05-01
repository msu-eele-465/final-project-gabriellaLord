#include <msp430.h>
#include <stdint.h>
#include <math.h>

#define SAMPLE_SIZE 50
#define SPEED_OF_SOUND 343.0f
#define MIC_DISTANCE 0.26f
#define FS 48000.0f
#define PI 3.1415926f

float crossCorr[2 * SAMPLE_SIZE - 1];
int lags[2 * SAMPLE_SIZE - 1];
volatile float aoa;

float rightRow[SAMPLE_SIZE] = {
    -2.1384e-07,-3.0917e-08,-7.7229e-07,-1.6274e-08,-1.1641e-06,3.7442e-09,-1.2171e-06,2.2525e-08,
    -8.2987e-07,3.3868e-08,-1.3124e-08,3.4024e-08,1.0929e-06,2.2943e-08,2.2277e-06,4.2846e-09,
    3.0359e-06,-1.5789e-08,3.0644e-06,-3.0647e-08,1.5351e-06,-3.5383e-08,-5.2557e-06,-2.8434e-08,
    3.7072e-05,-2.2758e-08,9.2267e-06,-1.5806e-09,4.4396e-06,4.3025e-08,1.5836e-06,3.3381e-08,
    -3.2845e-07,1.6322e-08,-1.4246e-06,3.152e-08,-1.7741e-06,7.7682e-09,-1.522e-06,-3.7298e-08,
    -8.901e-07,-2.909e-08,-1.3211e-07,-1.9647e-08,5.1789e-07,-3.9126e-08,8.9525e-07,-1.3861e-08,
    9.3958e-07,3.0321e-08
};

float leftRow[SAMPLE_SIZE] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    -5.5765e-11,-5.2321e-10,-9.4263e-11,-2.2981e-10,8.5175e-11,1.5797e-10,-1.097e-10,-1.1544e-10,
    1.4495e-09,1.583e-09,-5.0365e-10,-1.4142e-09,1.8965e-09,-1.0096e-09,-2.2691e-09,3.8296e-09,
    -5.1217e-10,2.8944e-09,4.4063e-09,1.2546e-09,2.3658e-11,5.9089e-09,2.0249e-09,-5.7925e-09,
    2.2007e-10,-9.7276e-10
};

void system_init(void) {
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;     // Enable GPIO
}

float my_fabs(float x) {
    return (x < 0) ? -x : x;
}

float my_acos(float x) {
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;

    float x2 = x * x;
    float x3 = x2 * x;
    float x5 = x3 * x2;
    float x7 = x5 * x2;

    float result = (PI / 2.0f)
                   - x
                   - (x3 / 6.0f)
                   - (3.0f * x5 / 40.0f)
                   - (5.0f * x7 / 112.0f);

    return result;
}


int find_max_index(const float *arr, int len) {
    int maxIndex = 0;
    float maxVal = my_fabs(arr[0]);
    int i;
    for (i = 1; i < len; i++) {
        float val = my_fabs(arr[i]);
        if (val > maxVal) {
            maxVal = val;
            maxIndex = i;
        }
    }
    return maxIndex;
}

void xcorr(const float *x, const float *y, float *corr, int *lag) {
    int len = 2 * SAMPLE_SIZE - 1;
    int mid = SAMPLE_SIZE - 1;
    int i;
    int j;

    for (i = 0; i < len; i++) {
        int shift = i - mid;
        lag[i] = shift;
        corr[i] = 0.0f;

        for (j = 0; j < SAMPLE_SIZE; j++) {
            int k = j - shift;
            if (k >= 0 && k < SAMPLE_SIZE) {
                corr[i] += x[j] * y[k];
            }
        }
    }
}

void calculate_aoa(void) {
    int len = 2 * SAMPLE_SIZE - 1;
    xcorr(leftRow, rightRow, crossCorr, lags);

    int index = find_max_index(crossCorr, len);
    float timeDelay = (float)lags[index] / FS;

    float ratio = timeDelay * SPEED_OF_SOUND / MIC_DISTANCE;
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;

    aoa = my_acos(ratio) * 180.0f / PI;
    __no_operation(); // Place a breakpoint here to inspect `aoa`
}

int main(void) {
    system_init();
    calculate_aoa();

    while (1) {
        __no_operation();
    }
}
