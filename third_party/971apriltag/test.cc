#include <iostream>
#include <cuda_runtime.h>

// A simple CUDA kernel to verify the GPU compiler works
__global__ void testKernel() {
    printf("Hello from the GPU! (Thread %d)\n", threadIdx.x);
}

int main() {
    std::cout << "Hello from the CPU (AArch64)!" << std::endl;
    std::cout << "Hello from the CPU (AArch64)!" << std::endl;

    // Launch the kernel with 5 threads
    testKernel<<<1, 5>>>();

    // Check for errors and wait for the GPU to finish
    cudaError_t err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        std::cout << "CUDA Error: " << cudaGetErrorString(err) << std::endl;
        return 1;
    }

    std::cout << "GPU execution finished successfully!" << std::endl;
    return 0;
}
