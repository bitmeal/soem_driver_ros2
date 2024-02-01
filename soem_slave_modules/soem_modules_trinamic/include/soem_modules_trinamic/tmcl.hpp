#ifndef SOEM_MODULES_TRINAMIC__TMCL_HPP_
#define SOEM_MODULES_TRINAMIC__TMCL_HPP_

#include <cstdint>

namespace soem_slave_modules
{
    namespace trinamic
    {
        namespace TMCL
        {
            // Opcodes of all TMCL commands that can be used in direct mode
            enum class Command : uint8_t
            {
                // motion
                ROR = 1, // Rotate right
                ROL = 2, // Rotate left
                MST = 3, // Motor stop
                MVP = 4, // Move to position

                // config
                SAP = 5,   // Set axis parameter
                GAP = 6,   // Get axis parameter
                STAP = 7,  // Store axis parameter into EEPROM
                RSAP = 8,  // Restore axis parameter from EEPROM
                SGP = 9,   // Set global parameter
                GGP = 10,  // Get global parameter
                STGP = 11, // Store global parameter into EEPROM
                RSGP = 12, // Restore global parameter from EEPROM

                // reference search
                RFS = 13,

                // IO
                SIO = 14, // set output
                GIO = 15, // get input / output

                // coordinate motion
                SCO = 30,
                GCO = 31,
                CCO = 32,

                // Opcodes of TMCL control functions (to be used to run or abort a TMCL program in the module)
                APPL_STOP = 128,
                APPL_RUN = 129,
                APPL_RESET = 131,

                // module options
                FIRMWARE_VERSION = 136,
                FACTORY_RESET = 137
            };

            namespace Parameter
            {
                // Options for MVP command
                enum class MVP : uint8_t
                {
                    ABS = 0,
                    REL = 1,
                    COORD = 2
                };

                // Options for RFS command
                enum class RFS : uint8_t
                {
                    START = 0,
                    STOP = 1,
                    STATUS = 2
                };

            } // namespace trinamic::TMCL::Parameter

            enum class Status : uint8_t
            {
                SUCCESS = 100,             // Successfully executed, no error
                COMMAND_LOADED = 101,      // Command loaded into TMCL program EEPROM
                CHECKSUM_ERROR = 1,        // Wrong checksum
                INVALID_COMMAND = 2,       // Invalid command
                WRONG_TYPE = 3,            // Wrong type
                INVALID_VALUE = 4,         // Invalid value
                EEPROM_LOCKED = 5,         // Configuration EEPROM locked
                COMMAND_NOT_AVAILABLE = 6, // Command not available
                PARAMETER_PASSWORD_PROTECTED = 8
            };

        } // namespace trinamic::TMCL
    } // namespace trinamic
} // namespace soem_slave_modules

#endif // SOEM_MODULES_TRINAMIC__TMCL_HPP_
