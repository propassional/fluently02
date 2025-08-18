#pip install pycuda did not work:  Building wheel for pycuda (pyproject.toml) did not run successfully.

import pycuda.autoinit
import pycuda.driver as drv
import numpy as np
from pycuda.compiler import SourceModule

# Define a simple kernel
mod = SourceModule("""
__global__ void multiply_them(float *dest, float *a, float *b)
{
    const int i = threadIdx.x;
    dest[i] = a[i] * b[i];
}
""")

multiply_them = mod.get_function("multiply_them")

# Create some data
a = np.random.randn(400).astype(np.float32)
b = np.random.randn(400).astype(np.float32)

# Allocate memory on the GPU
dest = np.zeros_like(a)

# Run the kernel
multiply_them(
    drv.Out(dest), drv.In(a), drv.In(b),
    block=(400, 1, 1), grid=(1, 1))

print(dest)