//https://github.com/Rishikeshdaoo/Reverberator/blob/master/Reverberator/src/com/rishi/reverb/Reverberation.java
void comb_filter(float ** inSlot, 
                 float ** outSlot,
                 int buffer_size,
                 float delay_ms,
                 float decay_factor,
                 int sample_rate)
{
    // Calculate delay in samples from delay in ms and samples/second
    int delay_samples = (int) (delay_ms * (sample_rate/1000));
    // ToDo: more efficient copy
    for (int i = 0; i< buffer_size; i++){
        outSlot[0][i] = inSlot[0][i];
        outSlot[1][i] = inSlot[1][i];
    }
    for (int i = 0; i< buffer_size - delay_samples; i++){
        outSlot[0][i+delay_samples] = outSlot[0][i]*decay_factor;
        outSlot[1][i+delay_samples] = outSlot[1][i]*decay_factor;
    }
};

void all_pass_filter(float ** inSlot, 
                 float ** outSlot,
                 int buffer_size,
                 int sample_rate)
{
    // ToDo: understand calculation of decay factor and samples
    for (int i = 0; i< buffer_size; i++){
            outSlot[0][i] = inSlot[0][i];
            outSlot[1][i] = inSlot[1][i];

            if (i - delay_samples >= 0 ) {
                outSlot[0][i] += -decay_factor * outSlot[i-delay_samples];
                outSlot[1][i] += -decay_factor * outSlot[i-delay_samples];
            }

            if (i - delay_samples >= 1)
        }
    
}