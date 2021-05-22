Import("env", "projenv")
import stlink
import UARTupload
import opentx
import upload_via_esp8266_backpack
import esp_compress

platform = env.get('PIOPLATFORM', '')
stm = platform in ['ststm32']

target_name = env['PIOENV'].upper()
target_board = env.get('BOARD').upper()
print("PLATFORM : '%s'" % platform)
print("BUILD ENV: '%s'" % target_name)
print("BOARD:     '%s'" % target_board)

# don't overwrite if custom command defined
if stm and "$UPLOADER $UPLOADERFLAGS" in env.get('UPLOADCMD', '$UPLOADER $UPLOADERFLAGS'):
    if "TX_R9M" in target_name or "TX_ES915TX" in target_name or "TX_GHOST" in target_name:
        env.AddPostAction("buildprog", opentx.gen_elrs)
        env.AddPreAction("upload", opentx.gen_elrs)
        if "WIFI" in target_name:
            env.Replace(UPLOADCMD=upload_via_esp8266_backpack.on_upload)
        else:
            env.Replace(UPLOADCMD=stlink.on_upload)
    elif "_WIFI" in target_name:
        env.Replace(UPLOADCMD=upload_via_esp8266_backpack.on_upload)
    elif "_BF_PASSTHROUGH" in target_name or "_BF" in target_name:
        env.Replace(UPLOADCMD=UARTupload.on_upload)
    else: # "_STLINK"
        env.Replace(UPLOADCMD=stlink.on_upload)
elif platform in ['espressif8266']:
    env.AddPostAction("buildprog", esp_compress.compressFirmware)
    env.AddPreAction("${BUILD_DIR}/spiffs.bin",
                     [esp_compress.compress_files])
    env.AddPreAction("${BUILD_DIR}/${ESP8266_FS_IMAGE_NAME}.bin",
                     [esp_compress.compress_files])
    env.AddPostAction("${BUILD_DIR}/${ESP8266_FS_IMAGE_NAME}.bin",
                     [esp_compress.compress_fs_bin])
    if "_WIFI" in target_name:
        env.Replace(UPLOAD_PROTOCOL="custom")
        env.Replace(UPLOADCMD=upload_via_esp8266_backpack.on_upload)
else:
    print("*** PLATFORM: '%s'" % platform)

