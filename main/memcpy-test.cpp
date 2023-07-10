#include <stdlib.h>
#include <iostream>
#include <cstring>
#define BUFFER_SIZE 10

int main() {
    int16_t *input_buf = (int16_t *)calloc(BUFFER_SIZE,2);
    int16_t *process_buf = (int16_t *)calloc(BUFFER_SIZE,2);
    int16_t *output_buf = (int16_t *)calloc(BUFFER_SIZE,2);
    int delay_samples = 4;
    for (int i = 0; i< BUFFER_SIZE; i++){
        input_buf[i] = i+BUFFER_SIZE;
        process_buf[i] =i;
        output_buf[i] =99;
    }

    std::memcpy(&output_buf[0],&process_buf[BUFFER_SIZE-delay_samples],(size_t) delay_samples * 2);

    std::memcpy(&output_buf[delay_samples],&input_buf[0],(size_t)(2*(BUFFER_SIZE-delay_samples)));

    std::memcpy(&process_buf[0], &input_buf[0], (size_t)BUFFER_SIZE*2);

    for (int i = 0; i< BUFFER_SIZE; i++){
       printf("i: %d : %d \n", i, output_buf[i]);
        input_buf[i] = i+2*BUFFER_SIZE;
    }

    std::memcpy(&output_buf[0],&process_buf[BUFFER_SIZE-delay_samples],(size_t) delay_samples * 2);

    std::memcpy(&output_buf[delay_samples],&input_buf[0],(size_t)(2*(BUFFER_SIZE-delay_samples)));

    std::memcpy(&process_buf[0], &input_buf[0], (size_t)BUFFER_SIZE*2);

    for (int i = 0; i< BUFFER_SIZE; i++){
       printf("i: %d : %d \n", i, output_buf[i]);
        input_buf[i] = i+2*BUFFER_SIZE;
    }
    return 0;
}