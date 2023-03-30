/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Robert Taylor / Pegasus Imagery Ltd
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANDriver.h>

#include <AP_Param/AP_Param.h>

#include <AP_EFI/AP_EFI_Currawong_ECU.h>

#ifdef HAL_PERIPH_MINERVA_CAN_ENABLE

#define MINERVA_CAN_PROTOCOL AP_CANManager::Driver_Type_Scripting2 // We'll use scripting2 for now since this is a periph

#define MINERVA_MSG_RATE_HZ_MIN 1
#define MINERVA_MSG_RATE_HZ_MAX 500
#define MINERVA_MSG_RATE_HZ_DEFAULT 500

#define MINERVA_CAN_ECU_ID_DEFAULT 255


class AP_MinervaCAN : public AP_CANDriver
{
public:
    AP_MinervaCAN();
    ~AP_MinervaCAN();

    // Minerva message groups define the CAN id's needing translation
    enum class MessageGroup : uint8_t {
        SIMULATOR = 0x00,       // Simulator messages
        ECU_OUT = 0x08,         // Messages *from* an ECU Inherited from PiccoloCAN
        ECU_IN = 0x09,          // Message *to* an ECU Inherited from PiccoloCAN

    };

    struct CurrawongECU_Info_t {
        float command;
        bool newCommand;
    } _ecu_info;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_MinervaCAN);

    static const struct AP_Param::GroupInfo var_info[];

    // Return MinervaCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_MinervaCAN *get_mcan(uint8_t driver_index);

    // initialize MinervaCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // called from SRV_Channels
    void update();

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on succses
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);
    
#if HAL_EFI_CURRAWONG_ECU_ENABLED
    void send_ecu_messages(void);

    // interpret an ECU message received over CAN
    bool handle_ecu_message(AP_HAL::CANFrame &frame);

    void encodeHFE_ECU_ThrottleCommandPacket(AP_HAL::CANFrame &txFrame, float throttleCommand);
#endif

    bool _initialized;
    char _thread_name[16];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_EventHandle _event_handle;

    AP_Int16 _ecu_id;        //! ECU Node ID
    AP_Int16 _ecu_hz;       //! ECU update rate (Hz)

    HAL_Semaphore _telem_sem;
};

#endif // HAL_PICCOLO_CAN_ENABLE
