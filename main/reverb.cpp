#include <algorithm>
#include <cstdlib>
// Mono, could deconstruct float ** into two float * (outSlot[0], outSlot[1]) for stereo.
void comb_filter(float * inSlot, 
                 float * outSlot,
                 int buffer_size,
                 int sample_rate,
                 float delay_ms,
                 float gain)
{
    // Calculate delay in samples from delay in ms and samples/second
    int delay_samples = (int) (delay_ms * (sample_rate/1000));
    for (int i = 0; i<delay_samples; i++){
        // Output is zero until delay reached.
        outSlot[i] = 0;
    }
    for (int i = delay_samples; i< buffer_size; i++){
        outSlot[i] = inSlot[i-delay_samples] + gain * outSlot[i-delay_samples];
    }
};

void normalise_and_smooth(float * outSlot, int buffer_size){
    // Find maximum of outSlot, scale all values by this.
    float max = 0.0f;

    for (int i=0; i<buffer_size; i++){
        if(std::abs(outSlot[i]) > max){
            max = outSlot[i];
        }
    };
    // float max_absolute_pointer = std::max_element(outSlot[0], outSlot[buffer_size], [](float a, float b) {
    //     return std::abs(a) < std::abs(b);
    // });
    float value = outSlot[0];
    for (int i=0; i<buffer_size; i++){
        // Smooth by only adding the variation between values, and scaling by max.
        outSlot[i] = (value + (outSlot[i]-value))/ (max);
        value = outSlot[i];
    }
}

void all_pass_filter(float * inSlot, 
                 float * outSlot,
                 int buffer_size,
                 int sample_rate,
                 float delay_ms,
                 float gain)
{
    // Calculate delay in samples from delay in ms and samples/second
    int delay_samples = (int) (delay_ms * (sample_rate/1000));
    float gain_squared = 1 - gain*gain;
    float * intermediatSlot = new float[buffer_size];
    for (int i = 0; i< buffer_size; i++){
        if (i < delay_samples) {
            // Delayed samples not yet looped around, out = -g * in
            outSlot[i] = -gain * inSlot[i];
            intermediatSlot[i] = 0;
        } else {
            intermediatSlot[i] = inSlot[i-delay_samples] + gain*intermediatSlot[i-delay_samples];
            outSlot[i] = -gain * inSlot[i] + gain_squared * intermediatSlot[i];
        }
    }
    normalise_and_smooth(outSlot, buffer_size);
}

