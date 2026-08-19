import os
import time


JOB_FILE = "/sd/fp2_job.bin"
FP_BUILD_ID = 7200128
JOB_READ_RETRY_COUNT = 30
JOB_READ_RETRY_DELAY_MS = 50
PACKET_SIZE = 20


def valid_job(data):
    if (data is None or len(data) != PACKET_SIZE
            or data[0] != 0xA5 or data[1] != 0x5A
            or data[2] != 0xF2 or data[3] != 0x01):
        return False
    checksum = 0
    for value in data[:-1]:
        checksum ^= value
    return checksum == data[-1]


def read_job_type():
    for attempt in range(JOB_READ_RETRY_COUNT):
        try:
            with open(JOB_FILE, "rb") as handle:
                data = handle.read()
            if valid_job(data):
                return data[4], attempt + 1
        except Exception:
            pass
        time.sleep_ms(JOB_READ_RETRY_DELAY_MS)
    return 0, JOB_READ_RETRY_COUNT


job_type, read_attempts = read_job_type()
print("#FP2_BOOT id=%d job=0x%02X read=%d" %
      (FP_BUILD_ID, job_type, read_attempts))
if job_type == 0x42:
    import gc
    del valid_job
    del read_job_type
    del read_attempts
    gc.collect()
    import fp2_image_worker
elif job_type == 0x43:
    import gc
    del valid_job
    del read_job_type
    del read_attempts
    gc.collect()
    import fp2_digit_worker
else:
    if job_type != 0:
        try:
            os.remove(JOB_FILE)
        except Exception:
            pass
    import fp2_listener
