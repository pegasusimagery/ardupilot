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


#include <AP_HAL/AP_HAL.h>

#include "minervaCAN.h"
#ifdef HAL_PERIPH_MINERVA_CAN_ENABLE

#include <AP_Param/AP_Param.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <SRV_Channel/SRV_Channel.h>

#include <AP_EFI/AP_EFI_Currawong_ECU.h>

#include <AP_PiccoloCAN/piccolo_protocol/ECUPackets.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "MinervaCAN", fmt, ##args); } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

// table of user-configurable Minerva CAN bus parameters
const AP_Param::GroupInfo AP_MinervaCAN::var_info[] = {
#if AP_EFI_CURRAWONG_ECU_ENABLED
    // @Param: ECU_ID
    // @DisplayName: ECU Node ID
    // @Description: Node ID to send ECU throttle messages to. Set to zero to disable ECU throttle messages. Set to 255 to broadcast to all ECUs.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("ECU_ID", 5, AP_MinervaCAN, _ecu_id, MINERVA_CAN_ECU_ID_DEFAULT), // Since this is a piccoloCAN ECU we'll just use their definition

    // @Param: ECU_RT
    // @DisplayName: ECU command output rate
    // @Description: Output rate of ECU command messages
    // @Units: Hz
    // @User: Advanced
    // @Range: 1 500
    AP_GROUPINFO("ECU_RT", 6, AP_MinervaCAN, _ecu_hz, MINERVA_MSG_RATE_HZ_DEFAULT),
#endif
    AP_GROUPEND
};

AP_MinervaCAN::AP_MinervaCAN()
{
    AP_Param::setup_object_defaults(this, var_info);

    debug_can(AP_CANManager::LOG_INFO, "MinervaCAN: constructed\n\r");
}

AP_MinervaCAN::~AP_MinervaCAN() { }

AP_MinervaCAN *AP_MinervaCAN::get_mcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != MINERVA_CAN_PROTOCOL) {
        return nullptr;
    }

    return static_cast<AP_MinervaCAN*>(AP::can().get_driver(driver_index));
}

bool AP_MinervaCAN::add_interface(AP_HAL::CANIface* can_iface) {
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "MinervaCAN: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "MinervaCAN: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "MinervaCAN: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "MinervaCAN: Cannot add event handle\n\r");
        return false;
    }
    return true;
}

// initialize MinervaCAN bus
void AP_MinervaCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "MinervaCAN: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "MinervaCAN: already initialized\n\r");
        return;
    }
    snprintf(_thread_name, sizeof(_thread_name), "MinervaCAN_%u", driver_index);

    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MinervaCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "MinervaCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "MinervaCAN: init done\n\r");
}

// loop to send output to CAN devices in background thread
void AP_MinervaCAN::loop()
{
    AP_HAL::CANFrame txFrame {};
    AP_HAL::CANFrame rxFrame {};

#if HAL_EFI_CURRAWONG_ECU_ENABLED
    uint16_t ecu_tx_counter = 0;
#endif

    // CAN Frame ID components
    uint8_t frame_id_group;     // Minerva message group
    uint16_t frame_id_device;   // Device identifier

    while (true) {

        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "MinervaCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

#if HAL_EFI_CURRAWONG_ECU_ENABLED
        _ecu_hz.set(constrain_int16(_ecu_hz, MINERVA_MSG_RATE_HZ_MIN, MINERVA_MSG_RATE_HZ_MAX));

        uint16_t ecuCmdRateMs = 1000 / _ecu_hz;
#endif
        uint64_t timeout = AP_HAL::micros64() + 250ULL;

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);

#if HAL_EFI_CURRAWONG_ECU_ENABLED
        // Transmit ecu throttle commands at regular intervals
        if (ecu_tx_counter++ > ecuCmdRateMs) {
            ecu_tx_counter = 0;
            send_ecu_messages();
        }
#endif

        // Look for any message responses on the CAN bus
        while (read_frame(rxFrame, timeout)) {
            // Extract group and device ID values from the frame identifier
            frame_id_group = (rxFrame.id >> 24) & 0x1F;
            frame_id_device = (rxFrame.id >> 8) & 0xFF;
            (void)frame_id_device; // Get rid of unused warning
            // Only accept extended messages
            if ((rxFrame.id & AP_HAL::CANFrame::FlagEFF) == 0) {
                continue;
            }

            switch (MessageGroup(frame_id_group)) {
            case MessageGroup::ECU_OUT:
            #if HAL_EFI_CURRAWONG_ECU_ENABLED
                if (handle_ecu_message(rxFrame)) {
                    // Returns true if the message was successfully decoded
                }
            #endif
                break;
            default:
                break;
            }
        }
    }
}

// write frame on CAN bus, returns true on success
bool AP_MinervaCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout)
{
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "MinervaCAN: Driver not initialized for write_frame\n\r");
        return false;
    }

    bool read_select = false;
    bool write_select = true;
    
    bool ret =  _can_iface->select(read_select, write_select, &out_frame, timeout);

    if (!ret || !write_select) {
        return false;
    }

    return (_can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
}

// read frame on CAN bus, returns true on succses
bool AP_MinervaCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout)
{
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "MinervaCAN: Driver not initialized for read_frame\n\r");
        return false;
    }
    bool read_select = true;
    bool write_select = false;
    bool ret = _can_iface->select(read_select, write_select, nullptr, timeout);

    if (!ret || !read_select) {
        // No frame available
        return false;
    }

    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};

    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// called from SRV_Channels
void AP_MinervaCAN::update()
{
#if HAL_EFI_CURRAWONG_ECU_ENABLED
    if (_ecu_id != 0) {
        _ecu_info.command = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle); 
        _ecu_info.newCommand = true;
    }
#endif // HAL_EFI_CURRAWONG_ECU_ENABLED
}

#if HAL_EFI_CURRAWONG_ECU_ENABLED
void AP_MinervaCAN::send_ecu_messages(void)
{
    AP_HAL::CANFrame txFrame {};

    const uint64_t timeout = AP_HAL::micros64() + 1000ULL;

    // No ECU node id set, don't send anything
    if (_ecu_id == 0) {
        return;
    }

    if (_ecu_info.newCommand) {
        encodeHFE_ECU_ThrottleCommandPacket(txFrame, _ecu_info.command);
        txFrame.id |= (uint8_t) _ecu_id;

        _ecu_info.newCommand = false;

        write_frame(txFrame, timeout);
    }
}

bool AP_MinervaCAN::handle_ecu_message(AP_HAL::CANFrame &frame)
{
    // Get the ecu instance
    AP_EFI_Currawong_ECU* ecu = AP_EFI_Currawong_ECU::get_instance();
    if (ecu != nullptr) {
        return ecu->handle_message(frame);
    }
    return false;
}

/**
 * HFE ECUs use a different throttle command packet structure despite sharing the telemetry structure with piccoloCAN
 * This function creates the different structure. 
 * In this case, the HFE uses one byte for throttle where piccoloCAN uses two
*/
void AP_MinervaCAN::encodeHFE_ECU_ThrottleCommandPacket(AP_HAL::CANFrame &txFrame, float throttleCommand) {
    // HFE ECU accepts throttle from 0-255
    // scale the Throttle
    uint8_t scaledvalue = (uint8_t)(floorf((throttleCommand*255.0f/100.0f)+0.5f));
    txFrame.data[0] = scaledvalue;
    txFrame.data[1] = 0;


    uint32_t id = (((uint8_t) AP_MinervaCAN::MessageGroup::ECU_IN) << 24) |       // CAN Group ID
                  ((txFrame.id & 0xFF) << 16);                                    // Message ID
    id |= AP_HAL::CANFrame::FlagEFF;
    txFrame.id = id;
    txFrame.dlc = 2;
}
#endif // HAL_EFI_CURRAWONG_ECU_ENABLED

/* Minerva Glue Logic
 * The following functions are required by the auto-generated protogen code.
 */


//! \return the packet data pointer from the packet
uint8_t* getECUPacketData(void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (uint8_t*) frame->data;
}

//! \return the packet data pointer from the packet, const
const uint8_t* getECUPacketDataConst(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (const uint8_t*) frame->data;
}

//! \return the size of a packet from the packet header
int getECUPacketSize(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (int) frame->dlc;
}

//! \return the ID of a packet from the packet header
uint32_t getECUPacketID(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    // Extract the message ID field from the 29-bit ID
    return (uint32_t) ((frame->id >> 16) & 0xFF);
}

#endif // HAL_Minerva_CAN_ENABLE
