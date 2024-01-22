#include <cstdint>
#include <cstring>
#include <iostream>
#define BUFFER_SIZE 10


int main(){
    size_t delay_samples = 3;
    int16_t *input_buf = (int16_t *)calloc(BUFFER_SIZE,sizeof(int16_t));
    int16_t *process_buf = (int16_t *)calloc(3*BUFFER_SIZE,sizeof(int16_t));
    int16_t *output_buf = (int16_t *)calloc(BUFFER_SIZE,sizeof(int16_t));

    for (size_t j = 0; j < 2; j++)
    {
        std::cout<<"Run "<<j<<" input:"<<std::endl;
        for (size_t i = 0; i < BUFFER_SIZE; i++)
        {
            input_buf[i] = i + j * BUFFER_SIZE;
            std::cout<<input_buf[i]<<std::endl;
        }
        std::memcpy(&process_buf[0], &input_buf[0], BUFFER_SIZE*sizeof(int16_t));

        // Output (2-3 BUFFER_SIZE) is the final delay_samples of the last input buffer (1-BUFFER_SIZE)
        std::memcpy(&process_buf[2*BUFFER_SIZE], &process_buf[2*BUFFER_SIZE-delay_samples], delay_samples*sizeof(int16_t));
        // Plus the remaining samples from the latest buffer
        std::memcpy(&process_buf[2*BUFFER_SIZE+delay_samples], &process_buf[0], (BUFFER_SIZE-delay_samples)*sizeof(int16_t));
        // Finally back up the current input buffer to the previous input buffer location.
        std::memcpy(&process_buf[BUFFER_SIZE ] , &process_buf[0], BUFFER_SIZE*sizeof(int16_t));
        // Copy final frame to output.
        std::memcpy(&output_buf[0], &process_buf[2*BUFFER_SIZE], BUFFER_SIZE*sizeof(int16_t));
        std::cout<<"Run "<<j<<" process:"<<std::endl;
        for (size_t i = 0; i <3*BUFFER_SIZE; i++)
        {
            std::cout<<process_buf[i]<<std::endl;
        }
        std::cout<<"Run "<<j<<" output:"<<std::endl;
        for (size_t i = 0; i <BUFFER_SIZE; i++)
        {
            std::cout<<output_buf[i]<<std::endl;
        }
    }

}
