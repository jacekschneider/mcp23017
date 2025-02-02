import gpiod
import os
import time

# Constants
MCP23017_LABEL = "mcp23017"
OUTPUT_BANK = range(0, 8)  # GPA0-GPA7 (as OUTPUTs)
INPUT_BANK = range(8, 16)  # GPB0-GPB7 (as INPUTs)

def find_gpiochip(label):
    """Find the gpiochip device with the given label by scanning /sys/class/gpio/"""
    gpio_base_path = "/sys/class/gpio/"

    # List all gpiochip entries
    for entry in os.listdir(gpio_base_path):
        if entry.startswith("gpiochip"):
            chip_path = os.path.join(gpio_base_path, entry, "label")
            try:
                with open(chip_path, "r") as f:
                    chip_label = f.read().strip()
                if chip_label == label:
                    chip_number = entry.replace("gpiochip", "")
                    return gpiod.Chip(f"/dev/gpiochip{chip_number}")
            except (FileNotFoundError, OSError):
                continue

    return None

def test_mcp23017():
    chip = find_gpiochip(MCP23017_LABEL)
    if not chip:
        print(f"Error: Could not find gpiochip with label '{MCP23017_LABEL}'")
        return
    
    print(f"Found MCP23017 at {chip.path}")

    # Get GPIO lines
    output_lines = chip.get_lines(OUTPUT_BANK)
    input_lines = chip.get_lines(INPUT_BANK)

    # Configure outputs
    output_lines.request(consumer="mcp23017_test", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0]*8)
    
    # Configure inputs
    input_lines.request(consumer="mcp23017_test", type=gpiod.LINE_REQ_DIR_IN)

    # Test output-to-input behavior
    for i in range(8):
        print(f"Testing GPIO {i} (Output) -> GPIO {i+8} (Input)")

        # Set output HIGH
        output_lines.set_values([1 if j == i else 0 for j in range(8)])
        time.sleep(0.1)  # Small delay for stabilization
        
        # Read inputs
        input_values = input_lines.get_values()
        expected_values = [1 if j == i else 0 for j in range(8)]

        print(f"  Expected: {expected_values}, Read: {input_values}")

        if input_values != expected_values:
            print(f"  ❌ ERROR: Mismatch on GPIO {i+8}")
        else:
            print(f"  ✅ PASS")

    # Cleanup
    output_lines.set_values([0]*8)  # Reset all outputs
    output_lines.release()
    input_lines.release()

    print("Test completed.")

if __name__ == "__main__":
    test_mcp23017()