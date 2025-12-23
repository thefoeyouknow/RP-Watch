# Run in the terminal with: python tools/elf2uf2.py build/waveshare_watch_test.elf
#!/usr/bin/env python3
import sys
import struct
import os
import shutil

# RP2040 UF2 Constants
UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END    = 0x0AB16F30
RP2040_FAMILY_ID = 0xE48BFF56
payload_size     = 256

def find_objcopy():
    # 1. Check global PATH
    path = shutil.which("arm-none-eabi-objcopy")
    if path: return f'"{path}"'

    # 2. Check Windows Pico SDK default location
    if sys.platform == "win32":
        home = os.path.expanduser("~")
        toolchain_base = os.path.join(home, ".pico-sdk", "toolchain")
        if os.path.exists(toolchain_base):
            # Find the version folder (e.g., 13_2_Rel1, 14_2_Rel1)
            for version in os.listdir(toolchain_base):
                bin_path = os.path.join(toolchain_base, version, "bin", "arm-none-eabi-objcopy.exe")
                if os.path.exists(bin_path):
                    return f'"{bin_path}"'
    
    return None

def convert_elf_to_uf2(elf_file, uf2_file):
    try:
        # We use objcopy to extract the binary data from the ELF first
        # If a corresponding .bin already exists, we can reuse it to avoid toolchain issues.
        bin_file = elf_file.replace(".elf", ".bin")
        
        # Determine the base address (Flash XIP start)
        base_address = 0x10000000

        # Locate the toolchain only if we need to generate the .bin
        if not os.path.exists(bin_file):
            objcopy_cmd = find_objcopy()
            if not objcopy_cmd:
                print("Warning: Could not find 'arm-none-eabi-objcopy'. Attempting to use existing .bin if present.")
            else:
                print(f"1. Extracting binary from {elf_file}...")
                res = os.system(f"{objcopy_cmd} -O binary {elf_file} {bin_file}")
                if res != 0:
                    print("Warning: arm-none-eabi-objcopy failed to run. If a prebuilt .bin exists, it will be used.")
        else:
            print(f"1. Reusing existing binary {bin_file}...")

        print(f"2. Converting {bin_file} to {uf2_file}...")
        
        with open(bin_file, "rb") as f:
            data = f.read()

        num_blocks = (len(data) + payload_size - 1) // payload_size
        
        with open(uf2_file, "wb") as f:
            for i in range(num_blocks):
                chunk = data[i*payload_size : (i+1)*payload_size]
                # Pad last chunk if needed
                if len(chunk) < payload_size:
                    chunk += b'\x00' * (payload_size - len(chunk))
                
                # Header construction
                # Magic0, Magic1, Flags, Target Addr, Payload Size, Block No, Num Blocks, FamilyID
                head = struct.pack("<IIIIIIII", 
                    UF2_MAGIC_START0, 
                    UF2_MAGIC_START1, 
                    0x00002000, # Flags: FamilyID present
                    base_address + i * 256, 
                    256, 
                    i, 
                    num_blocks, 
                    RP2040_FAMILY_ID
                )
                
                f.write(head)
                f.write(chunk)
                f.write(b'\x00' * (476 - 256)) # Padding
                f.write(struct.pack("<I", UF2_MAGIC_END)) # Footer magic

        print(f"Success! {uf2_file} created.")
        
        # Cleanup
        if os.path.exists(bin_file):
            os.remove(bin_file)

    except Exception as e:
        print(f"Error converting: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python elf2uf2.py <input.elf> [output.uf2]")
        # Auto-detect if run from build folder context
        files = [f for f in os.listdir('.') if f.endswith('.elf')]
        if len(files) > 0:
            print(f"Auto-detected {files[0]}...")
            convert_elf_to_uf2(files[0], files[0].replace(".elf", ".uf2"))
        else:
            sys.exit(1)
    else:
        elf = sys.argv[1]
        uf2 = sys.argv[2] if len(sys.argv) > 2 else elf.replace(".elf", ".uf2")
        convert_elf_to_uf2(elf, uf2)