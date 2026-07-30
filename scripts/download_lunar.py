import os
import time
import urllib.request

URLS = [
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0045B/DATA/ESM4/2020290/NAC/M1357459554LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0040C/DATA/ESM3/2019245/NAC/M1322065387LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0024/DATA/ESM2/2015235/NAC/M1195011983LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0013/DATA/ESM/2012349/NAC/M1110085056LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0011/DATA/SCI/2012105/NAC/M189013094LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0010/DATA/SCI/2012063/NAC/M185374654LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0010/DATA/SCI/2012027/NAC/M182272378LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0010/DATA/SCI/2011357/NAC/M179284423LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0010/DATA/SCI/2011357/NAC/M179270116LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0010/DATA/SCI/2011356/NAC/M179234388LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0010/DATA/SCI/2011356/NAC/M179177167LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011337/NAC/M177577659LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011337/NAC/M177570915LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011337/NAC/M177564146LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011337/NAC/M177557376LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011337/NAC/M177543812LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011337/NAC/M177509969LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011336/NAC/M177503185LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011336/NAC/M177482901LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011336/NAC/M177455825LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011318/NAC/M175943008LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011310/NAC/M175221021LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011310/NAC/M175180543LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011309/NAC/M175173809LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011309/NAC/M175140086LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011309/NAC/M175119854LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011309/NAC/M175099616LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011308/NAC/M175079372LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011308/NAC/M175072631LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0009/DATA/SCI/2011306/NAC/M174903953LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0008/DATA/SCI/2011256/NAC/M170581172LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0008/DATA/SCI/2011256/NAC/M170574381LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0008/DATA/SCI/2011256/NAC/M170567591LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0008/DATA/SCI/2011256/NAC/M170560802LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0008/DATA/SCI/2011256/NAC/M170554012LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0008/DATA/SCI/2011256/NAC/M170547222LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0008/DATA/SCI/2011256/NAC/M170540433LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0008/DATA/SCI/2011256/NAC/M170533643LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0008/DATA/SCI/2011256/NAC/M170526854LE.IMG",
    "http://pds.lroc.im-ldi.com/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0008/DATA/SCI/2011256/NAC/M170520064LE.IMG",
]

OUTPUT_DIR = "./lunar_images"
MAX_RETRIES = 3
TIMEOUT = 60

os.makedirs(OUTPUT_DIR, exist_ok=True)

succeeded = 0
failed = []

for i, url in enumerate(URLS[:40]):
    filename = url.split("/")[-1]
    dest = os.path.join(OUTPUT_DIR, filename)

    if os.path.exists(dest):
        print(f"[{i+1}/40] Already exists: {filename}")
        succeeded += 1
        continue

    for attempt in range(1, MAX_RETRIES + 1):
        try:
            print(f"[{i+1}/40] Downloading (attempt {attempt}): {filename}")
            urllib.request.urlretrieve(url, dest)
            size = os.path.getsize(dest)
            print(f"  -> {size:,} bytes saved")
            succeeded += 1
            break
        except Exception as e:
            print(f"  -> Error: {e}")
            if os.path.exists(dest):
                os.remove(dest)
            if attempt < MAX_RETRIES:
                time.sleep(2 ** attempt)
            else:
                failed.append(url)

print(f"\nDone: {succeeded} succeeded, {len(failed)} failed")
if failed:
    print("Failed URLs:")
    for u in failed:
        print(f"  {u}")
