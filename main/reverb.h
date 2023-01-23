void comb_filter(float * inSlot, 
                 float * outSlot,
                 int buffer_size,
                 int sample_rate,
                 float delay_ms,
                 float gain);

void normalise_and_smooth(float * outSlot, int buffer_size);
void all_pass_filter(float * inSlot, 
                 float * outSlot,
                 int buffer_size,
                 int sample_rate,
                 float delay_ms,
                 float gain);
 