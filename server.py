from typing import Optional

from fastapi import FastAPI
import json

from pydantic import BaseModel
##############
import datetime

app = FastAPI()

 

class Telemetri(BaseModel):
    class GPSSaatiSubClass(BaseModel):
        saat: int
        dakika: int
        saniye: int
        milisaniye: int
    takim_numarasi: int
    IHA_enlem: float
    IHA_boylam: float
    IHA_irtifa: float
    IHA_dikilme: int
    IHA_yonelme: int
    IHA_yatis: int
    IHA_hiz: int
    IHA_batarya: int
    IHA_otonom: bool
    IHA_kilitlenme: bool
    Hedef_merkez_X: int
    Hedef_merkez_Y: int
    Hedef_genislik: int
    Hedef_yukseklik: int
    #GPSSaati: Optional[GPSSaatiSubClass] = None

class koordinat(BaseModel):
    qrEnlem: float
    qrBoylam: float


class Kilitlenme(BaseModel):
    class BaslangicSubClass(BaseModel):
        saat: int
        dakika: int
        saniye: int
        milisaniye: int

    class BitisSubClass(BaseModel):
        saat: int
        dakika: int
        saniye: int
        milisaniye: int

    kilitlenmeBaslangicZamani: Optional[BaslangicSubClass] = None
    kilitlenmeBitisZamani: Optional[BitisSubClass] = None
    otonom_kilitlenme: bool


def saatAl():
    time = str(datetime.datetime.now())
    print(time)
    saat = time[11:23].split(":")
    return {"saat":saat[0],"dakika":saat[1],"saniye":saat[2].split(".")[0],"milisaniye":saat[2].split(".")[1]}

@app.get("/")
def read_root():
    return {"Docs": "/docs",
            "Redocs":"/redoc",
            "Owner":"@e"}

@app.get("/api/sunucusaati")
def sunucusaati():
    return saatAl()

@app.post("/api/kamikaze_bilgisi")
def kamikazeBilgi():
    return 0

@app.get("/api/qr_koordinati")
def koordinatAl():
    a = {
        "qrEnlem": 41.123456,
        "qrBoylam": 26.654987
    }
    return a

@app.post("/api/telemetri_gonder")
async def telemetri_gonder(item: Telemetri):
	a={
            "sistemSaati": saatAl(),
            "konumBilgileri": [

            {
            "takim_numarasi": 1,
            "iha_enlem": 40.231998,
            "iha_boylam": 29.0037,
            "iha_irtifa": 500,
            "iha_dikilme": 5,
            "iha_yonelme": 256,
            "iha_yatis": 0,
            "zaman_farki": 93
            },

            {
            "takim_numarasi": 2,
            "iha_enlem": 40.23126,
            "iha_boylam": 29.003631,
            "iha_irtifa": 190,
            "iha_dikilme": 5,
            "iha_yonelme": 256,
            "iha_yatis": 0,
            "zaman_farki": 74
            },

            {
            "takim_numarasi": 3,
            "iha_enlem": 40.243071,
            "iha_boylam": 29.003746,
            "iha_irtifa": 222.3,
            "iha_dikilme": 5,
            "iha_yonelme": 256,
            "iha_yatis": 0,
            "zaman_farki": 43

            }

            ]}
	return json.dumps(a)

@app.post("/api/giris")
def telemetri_gonder(item: Telemetri):
    return 0

@app.post("/api/kilitlenme_bilgisi")

def kilitlenme_bilgisi(item: Kilitlenme):
    return item

# @app.post("/api/giris")
# def giris():
    # return {}

# @app.get("/api/cikis")
# def cikis():
    # return {}
