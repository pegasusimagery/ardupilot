--[[
    Example for getting state of EFI and checking if the RPM is within tolerance
--]]


local efi_backend = nil

function do_check()
    local efi_state = efi_backend:get_state()
    local engine_speed = efi_state:engine_speed_rpm()
    if not engine_speed then
        return
    end
    if engine_speed > 7000 then
        gcs:send_text(2, "ECU RPM is too high!")
    end
end

function update()
    if not efi_backend then
        efi_backend = efi:get_backend(0)
        if not efi_backend then
            return
        end
    end
    do_check()
    return update, 100
end
gcs:send_text(6, "Started EFI speed check");
return update, 0