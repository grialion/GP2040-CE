#include "addons/gamepad_usb_host_listener.h"
#include "drivermanager.h"
#include "storagemanager.h"
#include "class/hid/hid.h"
#include "class/hid/hid_host.h"

// PS3 default output report for LED configuration
// Based on PS3 controller specification, this sets up the controller with default LED/rumble settings
static const uint8_t PS3_DEFAULT_OUT_REPORT[PS3_OUT_REPORT_SIZE] = {
    0x01, 0xff, 0x00, 0xff, 0x00,  // Report ID and rumble settings
    0x00, 0x00, 0x00, 0x00, 0x00,  // Rumble duration and LED bitmap (to be modified)
    0xff, 0x27, 0x10, 0x00, 0x32,  // LED 1: duration 0xff, period 0x27, on-time 0x10, off-time 0x00, brightness 0x32
    0xff, 0x27, 0x10, 0x00, 0x32,  // LED 2: same settings
    0xff, 0x27, 0x10, 0x00, 0x32,  // LED 3: same settings
    0xff, 0x27, 0x10, 0x00, 0x32,  // LED 4: same settings
    0x00, 0x00, 0x00, 0x00, 0x00,  // Reserved
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00
};

void GamepadUSBHostListener::setup() {
    _controller_host_enabled = false;
#if GAMEPAD_HOST_DEBUG
    stdio_init_all();
#endif
}

void GamepadUSBHostListener::process() {
    Gamepad *gamepad = Storage::getInstance().GetGamepad();
    gamepad->hasAnalogTriggers = true;
    gamepad->hasLeftAnalogStick = true;
    gamepad->hasRightAnalogStick = true;
    gamepad->state.dpad     |= _controller_host_state.dpad;
    gamepad->state.buttons  |= _controller_host_state.buttons;
    gamepad->state.lx       = _controller_host_state.lx;
    gamepad->state.ly       = _controller_host_state.ly;
    gamepad->state.rx       = _controller_host_state.rx;
    gamepad->state.ry       = _controller_host_state.ry;
    gamepad->state.rt       = _controller_host_state.rt;
    gamepad->state.lt       = _controller_host_state.lt;
}

void GamepadUSBHostListener::mount(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
    _controller_host_enabled = true;
    _controller_dev_addr = dev_addr;
    _controller_instance = instance;
    tuh_vid_pid_get(dev_addr, &controller_vid, &controller_pid);

#if GAMEPAD_HOST_DEBUG
    //printf("Mount: VID_%04x PID_%04x\n", controller_vid, controller_pid);
#endif

    uint16_t joystick_mid = GAMEPAD_JOYSTICK_MID;
    _controller_host_state.buttons = 0;
    _controller_host_state.dpad = 0;
    _controller_host_state.lx = joystick_mid;
    _controller_host_state.ly = joystick_mid;
    _controller_host_state.rx = joystick_mid;
    _controller_host_state.ry = joystick_mid;

    switch(controller_pid)
    {
        /* PS4/5 */
        // these require initialization
        case PS4_PRODUCT_ID:       // Razer Panthera
        case 0x00EE:               // Hori Minipad
        case PS4_WHEEL_PRODUCT_ID: // G29
        case 0xB67B:               // T-Flight
            init_ds4(desc_report, desc_len);
            break;
        // while these do not
        case DS4_ORG_PRODUCT_ID:   // Sony Dualshock 4 controller
        case DS4_PRODUCT_ID:       // Sony Dualshock 4 controller
            isDS4Identified = true;
            setup_ds4();
            break;
        case 0x0CE6:               // DualSense
            break;

        /* PS3 */
        case PS3_PRODUCT_ID:       // Sony DualShock 3 controller
            init_ps3();
            break;

        /* Switch Pro */
        case SWITCH_PRO_PRODUCT_ID: // Nintendo Switch Pro controller
            init_switchpro();
            break;

        case 0xC294:               // Driving Force or similar
            isDFInit = false;
            setup_df_wheel();
            break;
        case 0xC29A:
            isDFInit = true;
            break;

        /* Other */
        // these types do not have an identification step, at least for PS4
        case 0x9400:               // Google Stadia controller
        case 0x0510:               // pre-2015 Ultrakstik 360
        case 0x0511:               // Ultrakstik 360
        default:
            break;
    }
}

void GamepadUSBHostListener::unmount(uint8_t dev_addr) {
    _controller_host_enabled = false;
    controller_pid = 0x00;
    controller_vid = 0x00;
    _controller_dev_addr = 0;
    _controller_instance = 0;
    isDS4Identified = false;
    hasDS4DefReport = false;
    isPS3Initialized = false;
    ps3InitStage = 0;
    switchProInitState = SwitchProInitState::HANDSHAKE;
    switchProSequenceCounter = 0;
}

void GamepadUSBHostListener::report_received(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    // if a hid device hasn't been mounted
    if ( _controller_host_enabled == false ) return;

    // Interface protocol (hid_interface_protocol_enum_t)
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    // stop execution if a keyboard or mouse is mounted
    if ( itf_protocol == HID_ITF_PROTOCOL_KEYBOARD ) return;

    process_ctrlr_report(dev_addr, report, len);
}

void GamepadUSBHostListener::process_ctrlr_report(uint8_t dev_addr, uint8_t const* report, uint16_t len) {
#if GAMEPAD_HOST_DEBUG
    //printf("\033[1;0H\nHost (%d):\n", len);
    //for (uint8_t i = 0; i < len; i++) {
    //    printf("%02x ", report[i]);
    //    if (((i+1) % 16) == 0) printf("\n");
    //}
    //printf("----\n");
#endif

    switch(controller_pid)
    {
        case DS4_ORG_PRODUCT_ID:   // Sony Dualshock 4 controller
        case DS4_PRODUCT_ID:       // Sony Dualshock 4 controller
        case PS4_PRODUCT_ID:       // Razer Panthera
        case PS4_WHEEL_PRODUCT_ID: // G29
        case 0xB67B:               // T-Flight
        case 0x00EE:               // Hori Minipad
            if (isDS4Identified) {
                update_ds4();
                process_ds4(report, len);
            }
            break;
        case 0x0CE6:               // DualSense
            process_ds(report, len);
            break;
        case PS3_PRODUCT_ID:       // Sony DualShock 3 controller
            if (isPS3Initialized) {
                process_ps3(report, len);
            }
            break;
        case SWITCH_PRO_PRODUCT_ID: // Nintendo Switch Pro controller
            process_switchpro(report, len);
            break;
        case 0x9400:               // Google Stadia controller
            process_stadia(report, len);
            break;

        case 0xC294:               // Driving Force
            if (!isDFInit) setup_df_wheel();
            break;

        case 0xC29A:
            process_dfgt(report, len);
            break;

        case 0x0510:               // pre-2015 Ultrakstik 360
        case 0x0511:               // Ultrakstik 360
            process_ultrastik360(report, len);
            break;
        default:
            break;
    }
}

bool GamepadUSBHostListener::host_get_report(uint8_t report_id, void* report, uint16_t len) {
    awaiting_cb = true;
    return tuh_hid_get_report(_controller_dev_addr, _controller_instance, report_id, HID_REPORT_TYPE_FEATURE, report, len);
}

bool GamepadUSBHostListener::host_set_report(uint8_t report_id, void* report, uint16_t len) {
    awaiting_cb = true;
    return tuh_hid_set_report(_controller_dev_addr, _controller_instance, report_id, HID_REPORT_TYPE_FEATURE, report, len);
}

bool GamepadUSBHostListener::host_send_report(uint8_t report_id, void* report, uint16_t len) {
    return tuh_hid_send_report(_controller_dev_addr, _controller_instance, report_id, report, len);
}

void GamepadUSBHostListener::set_report_complete(uint8_t dev_addr, uint8_t instance, uint8_t report_id, uint8_t report_type, uint16_t len) {
    awaiting_cb = false;
}

void GamepadUSBHostListener::get_report_complete(uint8_t dev_addr, uint8_t instance, uint8_t report_id, uint8_t report_type, uint16_t len) {
#if GAMEPAD_HOST_DEBUG
    //printf("get_report_complete Report ID: %02x\n", report_id);
#endif
    if (!isDS4Identified) {
        switch (report_id) {
            case PS4AuthReport::PS4_DEFINITION:
                setup_ds4();
                break;
            default: 
                break;
        }
    }
    
    // Handle PS3 initialization stages - only for PS3 controllers
    if (!isPS3Initialized && controller_pid == PS3_PRODUCT_ID && report_id == PS3_GET_PAIRING_INFO) {
        setup_ps3();
    }
    
    awaiting_cb = false;
}

uint16_t GamepadUSBHostListener::map(uint8_t x, uint8_t in_min, uint8_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//check if different than 2
bool GamepadUSBHostListener::diff_than_2(uint8_t x, uint8_t y) {
    return (x - y > 2) || (y - x > 2);
}

// check if 2 reports are different enough
bool GamepadUSBHostListener::diff_report(PS4Report const* rpt1, PS4Report const* rpt2) {
    bool result;

    // x, y, z, rz must different than 2 to be counted
    result = diff_than_2(rpt1->leftStickX, rpt2->leftStickX) || diff_than_2(rpt1->leftStickY, rpt2->leftStickY) ||
            diff_than_2(rpt1->rightStickX, rpt2->rightStickX) || diff_than_2(rpt1->rightStickY, rpt2->rightStickY);

    // check the rest with mem compare
    result |= memcmp(&rpt1->rightStickY + 1, &rpt2->rightStickY + 1, sizeof(PS4Report)-6);

    return result;
}

void GamepadUSBHostListener::setup_ds4() {
    if (hasDS4DefReport) {
        // report came from the controller so copy the buffer
        memcpy(&ds4Config, report_buffer+1, sizeof(PS4ControllerConfig));
    }
    if ((ds4Config.hidUsage == 0x2721) || (ds4Config.hidUsage == 0x2127)) {
        isDS4Identified = true;
#if GAMEPAD_HOST_DEBUG
        //printf("PS4 controller details\n");
        //printf("----------------------\n");
        //printf("enableController: %d\n", ds4Config.features.enableController);
        //printf("enableMotion: %d\n", ds4Config.features.enableMotion);
        //printf("enableLED: %d\n", ds4Config.features.enableLED);
        //printf("enableRumble: %d\n", ds4Config.features.enableRumble);
        //printf("enableAnalog: %d\n", ds4Config.features.enableAnalog);
        //printf("enableUnknown0: %d\n", ds4Config.features.enableUnknown0);
        //printf("enableTouchpad: %d\n", ds4Config.features.enableTouchpad);
        //printf("enableUnknown1: %d\n", ds4Config.features.enableUnknown1);
#endif
    }
}

void GamepadUSBHostListener::init_ds4(const uint8_t* descReport, uint16_t descLen) {
    isDS4Identified = false;

    tuh_hid_report_info_t report_info[4];
    uint8_t report_count = tuh_hid_parse_report_descriptor(report_info, 4, descReport, descLen);
    for(uint8_t i = 0; i < report_count; i++) {
#if GAMEPAD_HOST_DEBUG
        //printf("Report: %02x, Usage: %04x, Usage Page: %04x\n", report_info[i].report_id, report_info[i].usage_page, report_info[i].usage);
#endif
        if (report_info[i].report_id == PS4AuthReport::PS4_DEFINITION) {
            // controller is some other type that's not a DS4, so parse the config
            memset(report_buffer, 0, PS4_ENDPOINT_SIZE);
            report_buffer[0] = PS4AuthReport::PS4_DEFINITION;
            host_get_report(PS4AuthReport::PS4_DEFINITION, report_buffer, 48);
            hasDS4DefReport = true;
            break;
        }
    }
    
    if (!hasDS4DefReport) {
        // no report found, DS4 or clone assume. use struct default data.
        //isDS4Identified = true;
    }
}

void GamepadUSBHostListener::update_ds4() {
#if GAMEPAD_HOST_USE_FEATURES
    Gamepad * gamepad = Storage::getInstance().GetProcessedGamepad();
    PS4FeatureOutputReport controller_output;

    memset(&controller_output, 0, sizeof(controller_output));

    controller_output.reportID = PS4AuthReport::PS4_SET_FEATURE_STATE;

    if (ds4Config.features.enableLED && gamepad->auxState.sensors.statusLight.enabled) {
        controller_output.enableUpdateLED = gamepad->auxState.sensors.statusLight.enabled;
        controller_output.ledRed = gamepad->auxState.sensors.statusLight.color.red;
        controller_output.ledGreen = gamepad->auxState.sensors.statusLight.color.green;
        controller_output.ledBlue = gamepad->auxState.sensors.statusLight.color.blue;
        controller_output.ledBlinkOn = gamepad->auxState.playerID.ledBlinkOn;
        controller_output.ledBlinkOff = gamepad->auxState.playerID.ledBlinkOff;
    }

    if (ds4Config.features.enableRumble) {
        gamepad->auxState.haptics.leftActuator.enabled = 1;
        gamepad->auxState.haptics.rightActuator.enabled = 1;
        controller_output.enableUpdateRumble = 1;
        controller_output.rumbleLeft = gamepad->auxState.haptics.leftActuator.intensity;
        controller_output.rumbleRight = gamepad->auxState.haptics.rightActuator.intensity;
    }

    void * report = &controller_output;
    uint16_t report_size = sizeof(controller_output)-1;

    tuh_hid_send_report(_controller_dev_addr, _controller_instance, 5, report+1, report_size);
#endif
}

void GamepadUSBHostListener::process_ds4(uint8_t const* report, uint16_t len) {
    PS4Report controller_report;

    // previous report used to compare for changes
    static PS4Report prev_report = { 0 };

    uint8_t const report_id = report[0];

    if (report_id == 1) {
        memcpy(&controller_report, report, sizeof(controller_report));

        if ( diff_report(&prev_report, &controller_report) ) {
            _controller_host_state.lx = map(controller_report.leftStickX, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.lt = controller_report.leftTrigger;
            _controller_host_state.rt = controller_report.rightTrigger;

            _controller_host_state.buttons = 0;
            if (controller_report.buttonTouchpad) _controller_host_state.buttons |= GAMEPAD_MASK_A2;
            if (controller_report.buttonSelect) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
            if (controller_report.buttonR3) _controller_host_state.buttons |= GAMEPAD_MASK_R3;
            if (controller_report.buttonL3) _controller_host_state.buttons |= GAMEPAD_MASK_L3;
            if (controller_report.buttonHome) _controller_host_state.buttons |= GAMEPAD_MASK_A1;
            if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            if (controller_report.buttonR1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
            if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
            if (controller_report.buttonNorth) _controller_host_state.buttons |= GAMEPAD_MASK_B4;
            if (controller_report.buttonEast) _controller_host_state.buttons |= GAMEPAD_MASK_B2;
            if (controller_report.buttonSouth) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
            if (controller_report.buttonWest) _controller_host_state.buttons |= GAMEPAD_MASK_B3;
            if (controller_report.buttonR2) _controller_host_state.buttons |= GAMEPAD_MASK_R2;
            if (controller_report.buttonL2) _controller_host_state.buttons |= GAMEPAD_MASK_L2;

            _controller_host_state.dpad = 0;
            if (controller_report.dpad == PS4_HAT_UP) _controller_host_state.dpad |= GAMEPAD_MASK_UP;
            if (controller_report.dpad == PS4_HAT_UPRIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_UP | GAMEPAD_MASK_RIGHT;
            if (controller_report.dpad == PS4_HAT_RIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
            if (controller_report.dpad == PS4_HAT_DOWNRIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT | GAMEPAD_MASK_DOWN;
            if (controller_report.dpad == PS4_HAT_DOWN) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
            if (controller_report.dpad == PS4_HAT_DOWNLEFT) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN | GAMEPAD_MASK_LEFT;
            if (controller_report.dpad == PS4_HAT_LEFT) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;
            if (controller_report.dpad == PS4_HAT_UPLEFT) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT | GAMEPAD_MASK_UP;
        }
    }

    prev_report = controller_report;
}

void GamepadUSBHostListener::process_ds(uint8_t const* report, uint16_t len) {
    DSReport controller_report;

    // previous report used to compare for changes
    static DSReport prev_ds_report = { 0 };

    uint8_t const report_id = report[0];

    if (report_id == 1) {
        memcpy(&controller_report, report, sizeof(controller_report));

        if ( prev_ds_report.reportCounter != controller_report.reportCounter ) {
            _controller_host_state.lx = map(controller_report.leftStickX, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.lt = controller_report.leftTrigger;
            _controller_host_state.rt = controller_report.rightTrigger;

            _controller_host_state.buttons = 0;
            if (controller_report.buttonTouchpad) _controller_host_state.buttons |= GAMEPAD_MASK_A2;
            if (controller_report.buttonSelect) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
            if (controller_report.buttonR3) _controller_host_state.buttons |= GAMEPAD_MASK_R3;
            if (controller_report.buttonL3) _controller_host_state.buttons |= GAMEPAD_MASK_L3;
            if (controller_report.buttonHome) _controller_host_state.buttons |= GAMEPAD_MASK_A1;
            if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            if (controller_report.buttonR1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
            if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
            if (controller_report.buttonNorth) _controller_host_state.buttons |= GAMEPAD_MASK_B4;
            if (controller_report.buttonEast) _controller_host_state.buttons |= GAMEPAD_MASK_B2;
            if (controller_report.buttonSouth) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
            if (controller_report.buttonWest) _controller_host_state.buttons |= GAMEPAD_MASK_B3;
            if (controller_report.buttonR2) _controller_host_state.buttons |= GAMEPAD_MASK_R2;
            if (controller_report.buttonL2) _controller_host_state.buttons |= GAMEPAD_MASK_L2;

            _controller_host_state.dpad = 0;
            if (controller_report.dpad == PS4_HAT_UP) _controller_host_state.dpad |= GAMEPAD_MASK_UP;
            if (controller_report.dpad == PS4_HAT_UPRIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_UP | GAMEPAD_MASK_RIGHT;
            if (controller_report.dpad == PS4_HAT_RIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
            if (controller_report.dpad == PS4_HAT_DOWNRIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT | GAMEPAD_MASK_DOWN;
            if (controller_report.dpad == PS4_HAT_DOWN) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
            if (controller_report.dpad == PS4_HAT_DOWNLEFT) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN | GAMEPAD_MASK_LEFT;
            if (controller_report.dpad == PS4_HAT_LEFT) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;
            if (controller_report.dpad == PS4_HAT_UPLEFT) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT | GAMEPAD_MASK_UP;
        }
    }

    prev_ds_report = controller_report;
}

void GamepadUSBHostListener::process_ps3(uint8_t const* report, uint16_t len) {
    PS3Report controller_report;

    // previous report used to compare for changes
    static PS3Report prev_report = { 0 };

    uint8_t const report_id = report[0];

    if (report_id == 1) {
        memcpy(&controller_report, report, sizeof(controller_report));

        // Only process if report has changed
        if (memcmp(&prev_report, &controller_report, sizeof(PS3Report)) != 0) {
            // Map analog sticks (PS3 uses 0x00-0xFF range with 0x80 as center)
            _controller_host_state.lx = map(controller_report.leftStickX, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            
            // PS3 has analog triggers (0x00 = unpressed, 0xFF = fully pressed)
            _controller_host_state.lt = controller_report.buttonL2Analog;
            _controller_host_state.rt = controller_report.buttonR2Analog;

            // Map buttons
            _controller_host_state.buttons = 0;
            if (controller_report.buttonTP) _controller_host_state.buttons |= GAMEPAD_MASK_A2;
            if (controller_report.buttonSelect) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
            if (controller_report.buttonR3) _controller_host_state.buttons |= GAMEPAD_MASK_R3;
            if (controller_report.buttonL3) _controller_host_state.buttons |= GAMEPAD_MASK_L3;
            if (controller_report.buttonPS) _controller_host_state.buttons |= GAMEPAD_MASK_A1;
            if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            if (controller_report.buttonR1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
            if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
            if (controller_report.buttonNorth) _controller_host_state.buttons |= GAMEPAD_MASK_B4;
            if (controller_report.buttonEast) _controller_host_state.buttons |= GAMEPAD_MASK_B2;
            if (controller_report.buttonSouth) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
            if (controller_report.buttonWest) _controller_host_state.buttons |= GAMEPAD_MASK_B3;
            if (controller_report.buttonR2) _controller_host_state.buttons |= GAMEPAD_MASK_R2;
            if (controller_report.buttonL2) _controller_host_state.buttons |= GAMEPAD_MASK_L2;

            // Map D-pad (PS3 uses individual bits, not a HAT value)
            _controller_host_state.dpad = 0;
            if (controller_report.dpadUp) _controller_host_state.dpad |= GAMEPAD_MASK_UP;
            if (controller_report.dpadDown) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
            if (controller_report.dpadLeft) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;
            if (controller_report.dpadRight) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
        }
    }

    prev_report = controller_report;
}

void GamepadUSBHostListener::init_ps3() {
    isPS3Initialized = false;
    ps3InitStage = 0;
    
    // Start PS3 initialization sequence by requesting pairing info (0xF2)
    memset(ps3_report_buffer, 0, sizeof(ps3_report_buffer));
    host_get_report(PS3_GET_PAIRING_INFO, ps3_report_buffer, PS3_INIT_REPORT_LEN_STAGE1);
}

void GamepadUSBHostListener::setup_ps3() {
    ps3InitStage++;
    
    if (ps3InitStage < PS3_INIT_STAGE_COUNT) {
        // Perform multiple GET_REPORT requests as part of initialization handshake
        uint16_t report_len = (ps3InitStage == 2) ? PS3_INIT_REPORT_LEN_STAGE3 : PS3_INIT_REPORT_LEN_STAGE2;
        memset(ps3_report_buffer, 0, sizeof(ps3_report_buffer));
        host_get_report(PS3_GET_PAIRING_INFO, ps3_report_buffer, report_len);
    } else {
        // Initialization complete, send output report to set LEDs
        isPS3Initialized = true;
        
        // Create output report based on default template
        uint8_t ps3_out_report[PS3_OUT_REPORT_SIZE];
        memcpy(ps3_out_report, PS3_DEFAULT_OUT_REPORT, PS3_OUT_REPORT_SIZE);
        
        // Set LED bitmap for player 1 (bit 1 = 0x02)
        ps3_out_report[9] = 0x02;
        
        // Send the output report to configure LEDs
        host_set_report(0x01, ps3_out_report, PS3_OUT_REPORT_SIZE);
    }
}

void GamepadUSBHostListener::process_stadia(uint8_t const* report, uint16_t len) {
    google_stadia_report_t controller_report;

    memcpy(&controller_report, report, sizeof(controller_report));

    _controller_host_state.lx = map(controller_report.GD_GamePadPointerX ,1,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.ly = map(controller_report.GD_GamePadPointerY,1 ,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.rx = map(controller_report.GD_GamePadPointerZ,1 ,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.ry = map(controller_report.GD_GamePadPointerRz,1 ,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.lt = controller_report.SIM_GamePadBrake;
    _controller_host_state.rt = controller_report.SIM_GamePadAccelerator;

    if (controller_report.BTN_GamePadButton18 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_A2;
    if (controller_report.BTN_GamePadButton17 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_A3;
    if (controller_report.BTN_GamePadButton11 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
    if (controller_report.BTN_GamePadButton15 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_R3;
    if (controller_report.BTN_GamePadButton14 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_L3;
    if (controller_report.BTN_GamePadButton13 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_A1;
    if (controller_report.BTN_GamePadButton12 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
    if (controller_report.BTN_GamePadButton8 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
    if (controller_report.BTN_GamePadButton7 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
    if (controller_report.BTN_GamePadButton5 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_B4;
    if (controller_report.BTN_GamePadButton4 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_B3;
    if (controller_report.BTN_GamePadButton2 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_B2;
    if (controller_report.BTN_GamePadButton1 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
    if (controller_report.BTN_GamePadButton19 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_R2;
    if (controller_report.BTN_GamePadButton20 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_L2;

    if (controller_report.GD_GamePadHatSwitch == 0) _controller_host_state.dpad |= GAMEPAD_MASK_UP;
    if (controller_report.GD_GamePadHatSwitch == 1) _controller_host_state.dpad |= GAMEPAD_MASK_UP | GAMEPAD_MASK_RIGHT;
    if (controller_report.GD_GamePadHatSwitch == 2) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
    if (controller_report.GD_GamePadHatSwitch == 3) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT | GAMEPAD_MASK_DOWN;
    if (controller_report.GD_GamePadHatSwitch == 4) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
    if (controller_report.GD_GamePadHatSwitch == 5) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN | GAMEPAD_MASK_LEFT;
    if (controller_report.GD_GamePadHatSwitch == 6) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;
    if (controller_report.GD_GamePadHatSwitch == 7) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT | GAMEPAD_MASK_UP;
}

void GamepadUSBHostListener::init_switchpro() {
    switchProInitState = SwitchProInitState::HANDSHAKE;
    switchProSequenceCounter = 0;
    
    // Start Switch Pro initialization with handshake
    SwitchProOutReport out_report;
    memset(&out_report, 0, sizeof(out_report));
    
    out_report.command = 0x80;  // HID command
    out_report.sequenceCounter = 0x02;  // HANDSHAKE
    
    host_send_report(0, &out_report, 2);
    switchProInitState = SwitchProInitState::TIMEOUT;
}

void GamepadUSBHostListener::process_switchpro(uint8_t const* report, uint16_t len) {
    // If not initialized, continue initialization
    if (switchProInitState != SwitchProInitState::DONE) {
        SwitchProOutReport out_report;
        memset(&out_report, 0, sizeof(out_report));
        
        // Set default rumble values
        out_report.rumbleL[0] = 0x00;
        out_report.rumbleL[1] = 0x01;
        out_report.rumbleL[2] = 0x40;
        out_report.rumbleL[3] = 0x40;
        out_report.rumbleR[0] = 0x00;
        out_report.rumbleR[1] = 0x01;
        out_report.rumbleR[2] = 0x40;
        out_report.rumbleR[3] = 0x40;
        
        uint8_t report_size = 10;
        
        switch (switchProInitState) {
            case SwitchProInitState::TIMEOUT:
                report_size = 2;
                out_report.command = 0x80;  // HID
                out_report.sequenceCounter = 0x04;  // DISABLE_TIMEOUT
                if (host_send_report(0, &out_report, report_size)) {
                    switchProInitState = SwitchProInitState::LED;
                }
                return;
                
            case SwitchProInitState::LED:
                report_size = 12;
                out_report.command = 0x01;  // AND_RUMBLE
                out_report.sequenceCounter = (switchProSequenceCounter++) & 0x0F;
                out_report.subCommand = 0x30;  // SET_PLAYER_LIGHTS
                out_report.subCommandArgs[0] = 0x01;  // Player 1 LED
                if (host_send_report(0, &out_report, report_size)) {
                    switchProInitState = SwitchProInitState::LED_HOME;
                }
                return;
                
            case SwitchProInitState::LED_HOME:
                report_size = 14;
                out_report.command = 0x01;  // AND_RUMBLE
                out_report.sequenceCounter = (switchProSequenceCounter++) & 0x0F;
                out_report.subCommand = 0x38;  // SET_HOME_LIGHT
                out_report.subCommandArgs[0] = (0 << 4) | 0xF;  // cycles and enable
                out_report.subCommandArgs[1] = (0xF << 4) | 0x0;  // intensity
                out_report.subCommandArgs[2] = (0xF << 4) | 0x0;  // mini cycle
                if (host_send_report(0, &out_report, report_size)) {
                    switchProInitState = SwitchProInitState::FULL_REPORT;
                }
                return;
                
            case SwitchProInitState::FULL_REPORT:
                report_size = 12;
                out_report.command = 0x01;  // AND_RUMBLE
                out_report.sequenceCounter = (switchProSequenceCounter++) & 0x0F;
                out_report.subCommand = 0x03;  // SET_MODE
                out_report.subCommandArgs[0] = 0x30;  // FULL_REPORT_MODE
                if (host_send_report(0, &out_report, report_size)) {
                    switchProInitState = SwitchProInitState::IMU;
                }
                return;
                
            case SwitchProInitState::IMU:
                report_size = 12;
                out_report.command = 0x01;  // AND_RUMBLE
                out_report.sequenceCounter = (switchProSequenceCounter++) & 0x0F;
                out_report.subCommand = 0x40;  // TOGGLE_IMU
                out_report.subCommandArgs[0] = 0x01;  // Enable
                if (host_send_report(0, &out_report, report_size)) {
                    switchProInitState = SwitchProInitState::DONE;
                    tuh_hid_receive_report(_controller_dev_addr, _controller_instance);
                }
                return;
                
            default:
                return;
        }
    }
    
    // Process input report
    const SwitchProInReport* controller_report = reinterpret_cast<const SwitchProInReport*>(report);
    static SwitchProInReport prev_report = { 0 };
    
    // Only process if report has changed (check button bytes)
    if (memcmp(prev_report.buttons, controller_report->buttons, 3) == 0) {
        tuh_hid_receive_report(_controller_dev_addr, _controller_instance);
        return;
    }
    
    // Extract 12-bit joystick values
    uint16_t joy_lx = controller_report->joysticks[0] | ((controller_report->joysticks[1] & 0xF) << 8);
    uint16_t joy_ly = (controller_report->joysticks[1] >> 4) | (controller_report->joysticks[2] << 4);
    uint16_t joy_rx = controller_report->joysticks[3] | ((controller_report->joysticks[4] & 0xF) << 8);
    uint16_t joy_ry = (controller_report->joysticks[4] >> 4) | (controller_report->joysticks[5] << 4);
    
    // Normalize from 12-bit (0-4095, center at SWITCH_PRO_JOYSTICK_CENTER) to int16 range
    // Apply multiplier to compensate for limited 12-bit range not covering full int16 range
    int16_t norm_lx = clamp_to_int16((int32_t)(joy_lx - SWITCH_PRO_JOYSTICK_CENTER) * SWITCH_PRO_JOYSTICK_MULTIPLIER);
    int16_t norm_ly = clamp_to_int16((int32_t)(joy_ly - SWITCH_PRO_JOYSTICK_CENTER) * SWITCH_PRO_JOYSTICK_MULTIPLIER);
    int16_t norm_rx = clamp_to_int16((int32_t)(joy_rx - SWITCH_PRO_JOYSTICK_CENTER) * SWITCH_PRO_JOYSTICK_MULTIPLIER);
    int16_t norm_ry = clamp_to_int16((int32_t)(joy_ry - SWITCH_PRO_JOYSTICK_CENTER) * SWITCH_PRO_JOYSTICK_MULTIPLIER);
    
    // Convert signed int16 to unsigned range and map to gamepad range
    _controller_host_state.lx = map(norm_lx + INT16_CENTER_OFFSET, 0, 65535, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.ly = map(norm_ly + INT16_CENTER_OFFSET, 0, 65535, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.rx = map(norm_rx + INT16_CENTER_OFFSET, 0, 65535, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.ry = map(norm_ry + INT16_CENTER_OFFSET, 0, 65535, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
    
    // Map buttons (buttons[0]: Y,X,B,A,SR,SL,R,ZR)
    _controller_host_state.buttons = 0;
    if (controller_report->buttons[0] & 0x01) _controller_host_state.buttons |= GAMEPAD_MASK_B3;  // Y -> X
    if (controller_report->buttons[0] & 0x02) _controller_host_state.buttons |= GAMEPAD_MASK_B4;  // X -> Y
    if (controller_report->buttons[0] & 0x04) _controller_host_state.buttons |= GAMEPAD_MASK_B1;  // B -> A
    if (controller_report->buttons[0] & 0x08) _controller_host_state.buttons |= GAMEPAD_MASK_B2;  // A -> B
    if (controller_report->buttons[0] & 0x40) _controller_host_state.buttons |= GAMEPAD_MASK_R1;  // R
    if (controller_report->buttons[0] & 0x80) _controller_host_state.buttons |= GAMEPAD_MASK_R2;  // ZR
    
    // buttons[1]: Minus,Plus,R3,L3,Home,Capture,dummy,charging
    if (controller_report->buttons[1] & 0x01) _controller_host_state.buttons |= GAMEPAD_MASK_S1;  // Minus -> Select
    if (controller_report->buttons[1] & 0x02) _controller_host_state.buttons |= GAMEPAD_MASK_S2;  // Plus -> Start
    if (controller_report->buttons[1] & 0x04) _controller_host_state.buttons |= GAMEPAD_MASK_L3;  // L3
    if (controller_report->buttons[1] & 0x08) _controller_host_state.buttons |= GAMEPAD_MASK_R3;  // R3
    if (controller_report->buttons[1] & 0x10) _controller_host_state.buttons |= GAMEPAD_MASK_A1;  // Home
    if (controller_report->buttons[1] & 0x20) _controller_host_state.buttons |= GAMEPAD_MASK_A2;  // Capture
    
    // buttons[2]: Down,Up,Right,Left,SL,SR,L,ZL
    if (controller_report->buttons[2] & 0x40) _controller_host_state.buttons |= GAMEPAD_MASK_L1;  // L
    if (controller_report->buttons[2] & 0x80) _controller_host_state.buttons |= GAMEPAD_MASK_L2;  // ZL
    
    // D-pad
    _controller_host_state.dpad = 0;
    if (controller_report->buttons[2] & 0x01) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
    if (controller_report->buttons[2] & 0x02) _controller_host_state.dpad |= GAMEPAD_MASK_UP;
    if (controller_report->buttons[2] & 0x04) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
    if (controller_report->buttons[2] & 0x08) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;
    
    // ZL/ZR are digital on Switch Pro (no analog triggers)
    _controller_host_state.lt = (controller_report->buttons[2] & 0x80) ? 255 : 0;
    _controller_host_state.rt = (controller_report->buttons[0] & 0x80) ? 255 : 0;
    
    tuh_hid_receive_report(_controller_dev_addr, _controller_instance);
    memcpy(&prev_report, controller_report, sizeof(SwitchProInReport));
}

void GamepadUSBHostListener::setup_df_wheel() {
    // send commands to see if can be reset to Driving Force GT mode for more compatibility
    uint8_t command[8] = {0xF8, 0x09, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00};
    uint16_t commandSize = sizeof(command);

    if (tuh_hid_send_report(_controller_dev_addr, _controller_instance, 0, command, commandSize)) {
        isDFInit = true;
    }
}

void GamepadUSBHostListener::process_dfgt(uint8_t const* report, uint16_t len) {
    PS3ReportAlt ps3Report;
    memcpy(&ps3Report, report, len);
#if GAMEPAD_HOST_DEBUG
    //printf("\033[2;0H");
    //for (uint8_t i = 0; i < len; i++) {
    //    printf("%02x ", report[i]);
    //    if (((i+1) % 16) == 0) printf("\n");
    //}
    //printf("\033[2;0H Ste:%5d Gas:%3d Brk:%3d", ps3Report.wheel.steeringWheel, ps3Report.wheel.gasPedal, ps3Report.wheel.brakePedal);
    //printf("\n-----\n");
    //printf("\033[2;0HDPad: %1d", ps3Report.wheel.dpadDirection);
    //printf("\033[3;0HWheel: %5d", ps3Report.wheel.steeringWheel);
    //printf("\033[4;0HGas: %3d", ps3Report.wheel.gasPedal);
    //printf("\033[5;0HBrake: %3d", ps3Report.wheel.brakePedal);
#endif
}

void GamepadUSBHostListener::process_ultrastik360(uint8_t const* report, uint16_t len) {

    ultrastik360_t controller_report;

    memcpy(&controller_report, report, sizeof(controller_report));

    _controller_host_state.lx = map(controller_report.GD_GamePadPointerX, 0, 255, GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.ly = map(controller_report.GD_GamePadPointerY, 0, 255, GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);

    if (controller_report.BTN_GamePadButton1 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
    if (controller_report.BTN_GamePadButton2 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_B2;
    if (controller_report.BTN_GamePadButton3 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_B3;
    if (controller_report.BTN_GamePadButton4 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_B4;
    if (controller_report.BTN_GamePadButton5 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
    if (controller_report.BTN_GamePadButton6 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_L2;
    if (controller_report.BTN_GamePadButton7 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
    if (controller_report.BTN_GamePadButton8 == 1) _controller_host_state.buttons |= GAMEPAD_MASK_R2;
}
