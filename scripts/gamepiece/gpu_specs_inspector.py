import torch

# Check if CUDA is available
print("CUDA available:", torch.cuda.is_available())

if torch.cuda.is_available():
    # Get number of GPUs
    num_gpus = torch.cuda.device_count()
    print(f"Number of CUDA devices: {num_gpus}")

    # Print info for each GPU
    for i in range(num_gpus):
        print(f"\n--- GPU {i} ---")
        print("Name:", torch.cuda.get_device_name(i))
        props = torch.cuda.get_device_properties(i)
        print(f"Total Memory: {props.total_memory / 1e9:.2f} GB")
        print(f"Compute Capability: {props.major}.{props.minor}")
        print(f"Multiprocessors: {props.multi_processor_count}")

    # Show which GPU is currently selected
    current = torch.cuda.current_device()
    print(f"\nCurrent device index: {current}")
    print("Current device name:", torch.cuda.get_device_name(current))
else:
    print("No CUDA devices detected.")
