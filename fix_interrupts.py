Import("env")
import os
from SCons.Script import DefaultEnvironment

# Like https://github.com/jdolinay/avr_debug/blob/master/doc/avr_debug.pdf says,
# We have to replace the WInterrupts.c of the Arduino core with a specialized WInterrupts.c
# from avr-debug, in case the library is used.

env = DefaultEnvironment()
platform = env.PioPlatform()
FRAMEWORK_DIR = platform.get_package_dir("framework-arduino-avr")

def replace_node_with_another(env, node):
    print("GOT INTO replace_node_with_another")
    return env.File(os.path.join("$PROJECT_DIR","patched_arduino", "WInterrupts.c"))

env.AddBuildMiddleware(
    replace_node_with_another,
    #"framework-arduino-avr/cores/arduino/WInterrupts.c"
    os.path.join(FRAMEWORK_DIR, "cores", "arduino", "WInterrupts.c")
)