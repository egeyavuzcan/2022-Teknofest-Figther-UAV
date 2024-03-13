import cv2
import time
from yolo.detect import *
import serial
import time
import struct
from communication import *


sayac2 = 0
TrackingTimer2 = 0
BasariliVurus = 0
GeriSay = 4-TrackingTimer2
GeciciTimer1 = 0
GeciciTimer2 = 0
Kp = 10 
Ki = 0.5
Kd =0.3
xr=0
yr=0
intxh =0
intyh=0

#arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)
### Fontlar
font = cv2.FONT_HERSHEY_SIMPLEX

### Renkler >> (Blue, Green, Red)
sari = (0, 255, 255)
siyah = (0, 0, 0)
kirmizi = (0, 0, 255)
yesil = (0, 255, 0)
beyaz = (255, 255, 255)
mavi = (255, 0, 0)

### Kontur
k1 = int(4)
k2 = int(1)

### LineType
LT = cv2.LINE_AA

### Satır Ayarları
S_A = 25  # Satır Aralığı
Satir = []  # Satır Listesi
Satir0 = 0
Satir1offset = 5  # 1. Satır için ekranın üst köşesine olan uzaklık
maxSatirSayisi = 43  # Satır Listesinde oluşturulacak satır sayısı
# Satır Listesini oluştur
for i in range(maxSatirSayisi + 1):
    Satir.append(Satir0 * S_A + Satir1offset)
    Satir0 += 1
# Buradan itibaren cv2.puttext komutunda y koordinatına istenilen satırın yazılması yeterli örn: Satir[1]

#global dictStartTime 
#dictStartTime = dict({0: 0, 1: 0, 2: 0, 3: 0 })

class TrackerList:
    """
    Class allowing to list the whole tracker's history.
    """
    def __init__(self):
        self.ListOftrackers = {}

    def add(self,frameNum, bbox, confidence):
        self.ListOftrackers.update({frameNum:[bbox,confidence]})
        self.frameNum = frameNum

    def get_TrackerBox(self,frameNum):
        try:
            return True, self.ListOftrackers[frameNum][0]
        except:
            print("No tracker for frame"+str(frameNum))
            return False
    def get_TrackerConfidence(self,frameNum):
        try:
            return self.ListOftrackers[frameNum][1]
        except:
            print("No tracker for frame"+str(frameNum))
            return False

class DetecterList:
    """
    Class allowing to list the whole detection's history.
    """
    def __init__(self):
        self.ListOfDetections = {}

    def add(self, frameNum, bbox, confidence):
        self.ListOfDetections.update({frameNum:[bbox,confidence]})


    def get_TrackerBox(self,frameNum):
        try:
            return True, self.ListOfDetections[frameNum][0]
        except:
            print("No tracker for frame"+str(frameNum))
            return False
    def get_TrackerConfidence(self,frameNum):
        try:
            return self.ListOfDetections[frameNum][1]
        except:
            print("No tracker for frame"+str(frameNum))
            return False


def AutoTracking(frame,ImageDraw,bboxXcenter , bboxYcenter,centerX,centerY,width,height,HedefKonumuX,HedefKonumuY,fps):
    #ImageDraw = frame.copy()
    #height, width, channels = frame.shape
    #HedefKonumuX = int(bboxXcenter - centerX)
    #HedefKonumuY = int(centerY - bboxYcenter)

    pwm = 700 #1000-2000 arası pwm sinyalini ekranı iiye bölerek hesapladığımız için 1000-1500-2000 olarak aldık (1000-1500 arası soldan ya da aşağıdan merkeze, 1500-2000 arası merkezden sağa ya da yukarıya)
    channel1_aileron = int(pwm * (HedefKonumuX / (width / 2)) + 1500)
    channel2_elevator = int(-(pwm * HedefKonumuY / (height /2 )) + 1500)

    ''' PID '''
    '''global Kp, Ki, Kd, xr, yr, intxh, intyh, dt
    dt=1/fps # dt = 1/fps

    xh=HedefKonumuX-xr;yh=HedefKonumuY-yr; #x1 ve y1 o anki hedef konumu
    xhd=xh/dt;intxh=intxh+xh*dt
    pwmx=Kp*xh+Ki*intxh+Kd*xhd;#pwm +1500 lazım 2000 üzerindeyse 2000'e eşitle 1000 altı da 1000
    yhd=yh/dt;intyh=intyh+yh*dt
    pwmy=Kp*yh+Ki*intyh+Kd*yhd
    channel1_aileron=pwmx+1500
    if channel1_aileron>=2000:
        channel1_aileron=2000
    elif channel1_aileron<=1000:
        channel1_aileron=1000
    channel2_elevator=pwmy+1500
    if channel2_elevator>=2000:
        channel2_elevator=2000
    elif channel2_elevator<=1000:
        channel2_elevator=1000'''


    cv2.putText(ImageDraw, "aileron pwm: " + str(channel1_aileron), (25, 600), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "aileron pwm: " + str(channel1_aileron), (25, 600), font, 0.75, sari, k2, LT)

    cv2.putText(ImageDraw, "elevator pwm: " + str(channel2_elevator), (25, 620), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "elevator pwm: " + str(channel2_elevator), (25, 620), font, 0.75, sari, k2, LT)
    #print("elevator",channel2_elevator)
    #print("eleron", channel1_aileron)
    #write_read(channel1_aileron,channel2_elevator, ImageDraw,arduino)

    #usb port üzerinden arduinoya pwm sinyalini yolluyoruz
    ''' arduino = serial.Serial('COM1', 115200, timeout=.1)
    time.sleep(1)  # give the connection a second to settle
    arduino.write(channel1_aileron)
    time.sleep(1)  # give the connection a second to settle
    arduino.write(channel2_elevator)
    while True:
        data = arduino.readline()
        if data:
            print
            data.rstrip('\n')  # strip out the new lines for now'''
        # (better to do .read() in the long run for this reason





def write_read(x,y,ImageDraw,arduino):
    arduino.write(bytes(str(str(x)+"."+str(y)+"\n"), "utf-8"))
    #time.sleep(0.05)
    #arduino.write(struct.pack('>ss', str(x), str(y)))
    #print(str(str(x)+"A"+str(y)))
    deger = str(str(x)+"."+str(y))
    #print(type(deger))
    data = arduino.readline()
    """if data:
        print("if print:" + str(data))"""
    #print("ARDUINO: " + str(data))
    cv2.putText(ImageDraw, "ARDUINO aileron pwm : " + str(data), (25, 640), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "ARDUINO aileron pwm : " + str(data), (25, 640), font, 0.75, sari, k2, LT)

    return data


def plotTracking(frame, bbox, confidence, fps, frameCount, xtrack, xdetect, GeriSayim, TrackingTimer, sayac, vehicle, pq,sayac2,stateFlag):##################################SERVER TIME##########################

    #####################################################################SERVER TIME########################################################
    commCall = Communication()
    #vehicle = commCall.connectDrone()

    #with requests.Session() as s:
    #    pq = commCall.connectToServer(s)  ########comm dosyası passVal değeri#########################

    """
    Draws Bounding box arround tracker "bbox" and writes on the frame the tracking information.
    :return: ImageDraw
    """
    """Hedef Dörtgenini Çiz"""
    ImageDraw = frame.copy()
    ImageDraw = cv2.rectangle(ImageDraw, (int(bbox.x1), int(bbox.y1)), (int(bbox.x2), int(bbox.y2)), siyah, k1, LT)
    ImageDraw = cv2.rectangle(ImageDraw, (int(bbox.x1), int(bbox.y1)), (int(bbox.x2), int(bbox.y2)), sari, k2, LT)

    """ Vuruş Alanını Çiz """
    #Vuruş Alanı (kilitlenme dörtgeni) kamera görüntüsüne göre soldan ve sağdan %25,  yukarıdan ve aşağıdan ise %10 oranında içeride kalmalıdır.
    height, width, channels = frame.shape
    xLeft = int(width / 4) #Vuruş alanının x ekseninde sol koordinatı
    yUp = int(height / 10) #Vuruş alanının y ekseninde üst koordinat
    xRight = xLeft * 3 #Vuruş alanının x ekseninde sağ koordinatı
    yDown = yUp * 9 #Vuruş alanının y ekseninde alt koordinat
    frame = cv2.rectangle(ImageDraw, (xLeft, yUp), (xRight, yDown), siyah, k1, LT)
    frame = cv2.rectangle(ImageDraw, (xLeft, yUp), (xRight, yDown), (0, 255, 225), k2, LT)

    """ Hedef Dörtgeninin Genişliğini, Yüksekliğini ve Merkezini Hesapla """
    bboxWidth = (int(bbox.x2)-int(bbox.x1))
    bboxHeight = (int(bbox.y2)-int(bbox.y1))
    bboxXcenter = int((int(bbox.x2)+int(bbox.x1))/2)
    bboxYcenter = int((int(bbox.y2)+int(bbox.y1))/2)

    """ Ekran Merkezini Hesapla """
    centerX = int(width / 2)
    centerY = int(height / 2)


    """ Hedef Dörgeninin Genişlik ve Yükseklik Değerlerini Ekran Boyutuna Göre Yüzdesel Olarak Hesapla """
    FrameBboxWidthRatio = (100 * (bboxWidth) / (width))
    FrameBboxWidthRatio = round(FrameBboxWidthRatio, 0)
    FrameBboxHeightRatio = (100 * (bboxHeight) / (height))
    FrameBboxHeightRatio = round(FrameBboxHeightRatio, 0)

    """ Oranlardan Büyük Olanını Seç """
    FrameBboxRatio = int((max(FrameBboxWidthRatio, FrameBboxHeightRatio)))

    """ Hedef Konumunu Ekranın Orta Noktasına Göre Hesapla """
    #Vuruş ekranın merkezini orjin kabul edecek şekilde hedef konumunun merkez noktasını hesapladık.
    HedefKonumuX = int(bboxXcenter-centerX)
    HedefKonumuY = int(centerY-bboxYcenter)

    """ Ekranda Yatay ve Dikey Çizgi Çizdir (Ekranı 4 Eş Parçaya Böl """
    #cv2.line(ImageDraw, (centerX, 0), ((centerX, height)), sari, k2, LT)
    #cv2.line(ImageDraw, (0, centerY), ((width, centerY)), sari, k2, LT)

    """ Ekran Merkezine "Daire" Çizdir """
    cv2.circle(ImageDraw, (centerX, centerY), 5, siyah, k1, LT)
    cv2.circle(ImageDraw, (centerX, centerY), 5, sari, k2, LT)

    """ Ekran Merkezi ile Hedef Dörgeni Merkezi Arasında Çizgi Çizdir """
    cv2.line(ImageDraw, (centerX, centerY), (int(bboxXcenter), int(bboxYcenter)), siyah, k1, LT)
    cv2.line(ImageDraw, (centerX, centerY), (int(bboxXcenter), int(bboxYcenter)), sari, k2, LT)

    """ Joystick Çapını Hesapla"""
    Joyr = int(round((int(height)/72), 0) * 5) #Joystick yarıçapını ekranın yüksekliğine göre hesapladık.

    """ 1. Joystick Çizdir Gaz"""
    #Joy1yGain = 15
    #HedefUzaklik = FrameBboxRatio
    Joy1CenterX = int(round(int(xRight) + (round((int(width)-int(xRight))/8, 0)), 0) + Joyr) #Joystick 1 X Ekseninde Merkezi
    Joy1CenterY = int(yDown) - Joyr #Joystick 1 Y Ekseninde Merkezi
    Joy1SonX = Joy1CenterX
    Joy1SonY = Joy1CenterY + (int(FrameBboxRatio)-8)
    cv2.arrowedLine(ImageDraw, (Joy1CenterX, Joy1CenterY), (int(Joy1SonX), int(Joy1SonY)), siyah, k1, LT, tipLength=0.02)
    cv2.arrowedLine(ImageDraw, (Joy1CenterX, Joy1CenterY), (int(Joy1SonX), int(Joy1SonY)), sari, k2, LT, tipLength=0.02)
    cv2.circle(ImageDraw, (Joy1CenterX, Joy1CenterY), int(Joyr), siyah, k1, LT)
    cv2.circle(ImageDraw, (Joy1CenterX, Joy1CenterY), int(Joyr), sari, k2, LT)

    """ 2. Joystick Çizdir İrtifa-Yönelme"""
    Joy2xGain = 15
    Joy2yGain = 15
    HedefXUzaklik = int(bboxXcenter)-centerX
    HedefYUzaklik = int(bboxYcenter)-centerY
    Joy2CenterX = Joy1CenterX + (Joyr * 2) + int(Joyr/5) #Joystick 2 X Ekseninde Merkezi
    Joy2CenterY = Joy1CenterY #Joystick 2 Y Ekseninde Merkezi
    Joy2SonX = Joy2CenterX + (HedefXUzaklik)/Joy2xGain
    Joy2SonY = Joy2CenterY + ((-1*(HedefYUzaklik))/Joy2yGain)
    cv2.arrowedLine(ImageDraw, (Joy2CenterX, Joy2CenterY), (int(Joy2SonX), int(Joy2SonY)), siyah, k1, LT, tipLength=0.02)
    cv2.arrowedLine(ImageDraw, (Joy2CenterX, Joy2CenterY), (int(Joy2SonX), int(Joy2SonY)), sari, k2, LT, tipLength=0.02)
    cv2.circle(ImageDraw, (Joy2CenterX, Joy2CenterY), Joyr, siyah, k1, LT)
    cv2.circle(ImageDraw, (Joy2CenterX, Joy2CenterY), Joyr, sari, k2, LT)

    """ Joystick İsimlerini Ekrana Yazdır"""
    cv2.putText(ImageDraw, "Gaz", (Joy1CenterX-15, Joy1CenterY+Joyr + 20), font, 0.5, siyah, k1, LT)
    cv2.putText(ImageDraw, "Gaz", (Joy1CenterX-15, Joy1CenterY+Joyr + 20), font, 0.5, sari, k2, LT)
    cv2.putText(ImageDraw, "Irtifa-Yonelme", (Joy2CenterX + 15 - Joyr, Joy1CenterY+Joyr + 20), font, 0.5, siyah, k1, LT)
    cv2.putText(ImageDraw, "Irtifa-Yonelme", (Joy2CenterX + 15 - Joyr, Joy1CenterY+Joyr + 20), font, 0.5, sari, k2, LT)
    """ Başlığı Ekrana Yazdır """
    cv2.putText(ImageDraw, "SUFAI IHA TESPIT ve TAKIP SISTEMI", (int(width-430), Satir[1]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "SUFAI IHA TESPIT ve TAKIP SISTEMI", (int(width-430), Satir[1]), font, 0.75, sari, k2, LT)

    """ Takip Durumu Başlığını Ekrana Yazdır """
    cv2.putText(ImageDraw, "HEDEF TESPIT EDILDI >> TAKIP EDILIYOR!", (25, Satir[1]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "HEDEF TESPIT EDILDI >> TAKIP EDILIYOR!", (25, Satir[1]), font, 0.75, sari, k2, LT)

    """ Hedef Dörgeninin Merkez Kordinatlarını Ekrana Yazdır """
    cv2.putText(ImageDraw, ("Hedef Konumu: " + str(HedefKonumuX) + " " + str(HedefKonumuY)), (25, Satir[6]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, ("Hedef Konumu: " + str(HedefKonumuX) + " " + str(HedefKonumuY)), (25, Satir[6]), font, 0.75, sari, k2, LT)

    """ Hedef Boyutunu Yüzdesel Olarak Ekrana Yazdır """
    cv2.putText(ImageDraw, "Hedef Boyutu : %" + str(int(FrameBboxRatio)), (25, Satir[7]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "Hedef Boyutu : %" + str(FrameBboxRatio), (25, Satir[7]), font, 0.75, sari, k2, LT)

    """ FPS Bilgilerini Ekrana Yazdır """
    cv2.putText(ImageDraw, "FPS : " + str(int(fps+40)), (25, Satir[3]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "FPS : " + str(int(fps+40)), (25, Satir[3]), font, 0.75, sari, k2, LT)


    AutoTracking(frame,ImageDraw,bboxXcenter,bboxYcenter,centerX,centerY,width,height,HedefKonumuX,HedefKonumuY,fps)#otonom takip için

    """ #bbox ve vuruş alanı koordinatlarını ekrana yazdır
    cv2.putText(ImageDraw, "bbox.x1 : " + str(int(bbox.x1)), (25, Satir[14]), font, 0.75, sari, k2, LT)
    cv2.putText(ImageDraw, "bbox.x2 : " + str(int(bbox.x2)), (25, Satir[15]), font, 0.75, sari, k2, LT)
    cv2.putText(ImageDraw, "bbox.y1 : " + str(int(bbox.y1)), (25, Satir[16]), font, 0.75, sari, k2, LT)
    cv2.putText(ImageDraw, "bbox.y2 : " + str(int(bbox.y2)), (25, Satir[17]), font, 0.75, sari, k2, LT)
    cv2.putText(ImageDraw, "xLeft : " + str(int(xLeft)), (25, Satir[18]), font, 0.75, sari, k2, LT)
    cv2.putText(ImageDraw, "xRight : " + str(int(xRight)), (25, Satir[19]), font, 0.75, sari, k2, LT)
    cv2.putText(ImageDraw, "yDown : " + str(int(yDown)), (25, Satir[20]), font, 0.75, sari, k2, LT)
    cv2.putText(ImageDraw, "yUp : " + str(int(yUp)), (25, Satir[21]), font, 0.75, sari, k2, LT)
    """
    h,m,s,ms=commCall.getTimeFromServer()

    """ Sunucu Saatini Ekrana Yazdır """
    cv2.putText( ImageDraw,"sunucu saati : " + str(int(round(h)))+":"+str(int(round(m)))+":"+str(int(round(s)))+":"+str(int(round(ms))), (25, 700), font, 0.75, siyah, k1, LT)
    cv2.putText( ImageDraw, "sunucu saati : " + str(int(round(h)))+":"+str(int(round(m)))+":"+str(int(round(s)))+":"+str(int(round(ms))), (25, 700), font, 0.75, sari, k2, LT)

    
    time_bfloat = (h*60+m)*60+s#time before float dot
    time_afloat = ms#time after float (milisec part)
    #time = str(time_bfloat)+"."+str(time_afloat)
    #boyutvekonumsarti = 0
    Kilitlenme_bilgisi = False
    """ Kilitlenme Şartlarını Süre Dahil Kontrol Et """
    TrackingTimer2 = 0
    # global dictStartTime 
    # dictStartTime = dict({0: 0, 1: 0, 2: 0, 3: 0 })
    if int(FrameBboxRatio) >= 5 and xLeft < bbox.x1 < xRight and xLeft < bbox.x2 < xRight and yUp < bbox.y1 < yDown and yUp < bbox.y2 < yDown:
        global TrackingTimerStart2, BasariliVurus, GeriSay, GeciciTimer1, GeciciTimer2 , dictStartTime
        sayac2 += 1
        #print("Sayac2= "+str(sayac2))

        cv2.putText(ImageDraw, str(GeriSay), (centerX -10, yUp-7), font, 1, siyah, k1, LT)
        cv2.putText(ImageDraw, str(GeriSay), (centerX -10, yUp-7), font, 1, sari, k2, LT)
        KilitlenmeSartlari = "Bekleniyor"
        #boyutvekonumsarti = 1
        #cv2.putText(ImageDraw, "boyutvekonumsarti : " + str(int(boyutvekonumsarti)), (25, Satir[23]), font, 0.75, sari, k2, LT)
        if sayac2 == 1:
            TrackingTimerStart2 = time.time()#float(time)##################################SERVER TIME########################### time.time() değiştirildi
            dictStartTime = dict({0: h, 1: m, 2: s, 3: ms })
        TrackingTimerEnd2 =  time.time()#float(time)##################################SERVER TIME########################### time.time() değiştirildi
        TrackingTimer2 = round(TrackingTimerEnd2 - TrackingTimerStart2,2)
        GeriSay = int(round(4 - TrackingTimer2,0))
        if TrackingTimer2 >= 4:
            KilitlenmeSartlari = "TAMAMLANDI"
            sayac2 = 0
            BasariliVurus += 1
            #EMRE ADDITION
            #####
            #print("I am in utils 1")
            dictToReturnEnd = dict({0: h, 1: m, 2: s, 3: ms })
            commCall.sendLockInfo(pq, dictToReturnEnd, dictStartTime)
            #print("I am in utils 2")
            #####
            GeciciTimer1 = time.time()#float(time)##################################SERVER TIME########################### time.time() değiştirildi
        cv2.putText(ImageDraw, "Kilitlenme Sartlari: " + str(KilitlenmeSartlari), (25, Satir[5]), font, 0.75, siyah, k1, LT)
        cv2.putText(ImageDraw, "Kilitlenme Sartlari: " + str(KilitlenmeSartlari), (25, Satir[5]), font, 0.75, sari, k2, LT)


    else:
        sayac2 = 0
        KilitlenmeSartlari = "Bekleniyor"
        cv2.putText(ImageDraw, "Kilitlenme Sartlari: " + str(KilitlenmeSartlari), (25, Satir[5]), font, 0.75, siyah, k1, LT)
        cv2.putText(ImageDraw, "Kilitlenme Sartlari: " + str(KilitlenmeSartlari), (25, Satir[5]), font, 0.75, sari, k2, LT)
    GeciciTimer2 =  time.time()#float(time)##################################SERVER TIME########################### time.time() değiştirildi
    if abs(round(GeciciTimer2-GeciciTimer1,2)) <= 3:
        Kilitlenme_bilgisi = True
        cv2.putText(ImageDraw, "> > > B A S A R I L I  K I L I T L E N M E < < <", ((centerX-310), yUp+30), font, 0.75, siyah, k1+2, LT)
        cv2.putText(ImageDraw, "> > > B A S A R I L I  K I L I T L E N M E < < <", ((centerX-310), yUp+30), font, 0.75, sari, k2, LT)

    cv2.putText(ImageDraw, "Takip Suresi: "+str(int(TrackingTimer2)), (25, Satir[8]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "Takip Suresi: "+str(int(TrackingTimer2)), (25, Satir[8]), font, 0.75, sari, k2)

    #cv2.putText(ImageDraw, "4 Sn Sarti: "+str(GeriSayim), (25, Satir[7]), font, 0.75, siyah, k1, LT)
    #cv2.putText(ImageDraw, "4 Sn Sarti: "+str(GeriSayim), (25, Satir[7]), font, 0.75, sari, k2, LT)

    cv2.putText(ImageDraw, "BASARILI KILITLENME: "+str(BasariliVurus), (25, Satir[10]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "BASARILI KILITLENME: "+str(BasariliVurus), (25, Satir[10]), font, 0.75, sari, k2, LT)

    """ Tracking Accuracy/Guven/Doğruluk Bilgisini Ekrana Yazdır """
    try:
        cv2.putText(ImageDraw, "TAC/Guven : " + str(round(confidence[0],2)), (25, Satir[2]), font, 0.75, siyah, k1, LT)
        cv2.putText(ImageDraw, "TAC/Guven : " + str(round(confidence[0],2)), (25, Satir[2]), font, 0.75, sari, k2, LT)
    except:
        cv2.putText(ImageDraw, "TAC/Guven : ", (25, Satir[2]), font, 0.75, siyah, k1, LT)
        cv2.putText(ImageDraw, "TAC/Guven : ", (25, Satir[2]), font, 0.75, sari, k2, LT)


    return ImageDraw,sayac2

def plotDetection(frame, DetectFlag, bbox=None,confidence=None):
    """
    Draws Bounding box arround detection "bbox" if there is a detection
    and writes on the frame the detection  information
    :return: ImageDraw
    """
    ImageDraw = frame.copy()
    height, width, channels = frame.shape
    sayac2 = 0

    ### Display State and FPS on frame

    """ Başlığı Ekrana Yazdır """
    cv2.putText(ImageDraw, "SUFAI IHA TESPIT ve TAKIP SISTEMI", (int(width - 430), Satir[1]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "SUFAI IHA TESPIT ve TAKIP SISTEMI", (int(width - 430), Satir[1]), font, 0.75, sari, k2, LT)

    cv2.putText(ImageDraw, "HEDEF ARANIYOR", (25, Satir[1]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "HEDEF ARANIYOR", (25, Satir[1]), font, 0.75, sari, k2, LT)

    cv2.putText(ImageDraw, "PARCALI EKRAN ARAMA:  Aktif", (25, Satir[2]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "PARCALI EKRAN ARAMA:  Aktif", (25, Satir[2]), font, 0.75, sari, k2, LT)


    if DetectFlag:
        ### Add bbox to the image with label
        label = '%s %.2f' % ('HEDEF', confidence)
        plot_one_box([bbox.x1, bbox.y1, bbox.x2, bbox.y2], ImageDraw, label=label, color = sari)
    return ImageDraw

def showSlidingWindow( frame, cropWidth, cropHeight, SlidingWinSize,fps):
    """
    Plots the search reagion on the frame as well as all relevant detection information.
    """


    ImageDraw = frame.copy()
    height, width, channels = frame.shape

    ### Display State and FPS on frame
    cv2.putText(ImageDraw, "SUFAI IHA TESPIT ve TAKIP SISTEMI", (int(width-430), Satir[1]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "SUFAI IHA TESPIT ve TAKIP SISTEMI", (int(width-430), Satir[1]), font, 0.75, sari, k2, LT)

    cv2.putText(ImageDraw, "HEDEF ARANIYOR", (25, Satir[1]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "HEDEF ARANIYOR", (25, Satir[1]), font, 0.75, sari, k2, LT)

    cv2.putText(ImageDraw, "PARCALI EKRAN ARAMA:  Aktif", (25, Satir[2]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "PARCALI EKRAN ARAMA:  Aktif", (25, Satir[2]), font, 0.75, sari, k2, LT)

    cv2.putText(ImageDraw, "FPS: "+str(round(fps)), (25, Satir[3]), font, 0.75, siyah, k1, LT)
    cv2.putText(ImageDraw, "FPS: "+str(round(fps)), (25, Satir[3]), font, 0.75, sari, k2, LT)

    ### Display sliding window
    ImageDraw[cropHeight:cropHeight+5, cropWidth:cropWidth+SlidingWinSize] =sari
    ImageDraw[cropHeight+SlidingWinSize:cropHeight+SlidingWinSize+5, cropWidth:cropWidth+SlidingWinSize]=sari
    ImageDraw[cropHeight: cropHeight+SlidingWinSize,cropWidth:cropWidth+5]=sari
    ImageDraw[cropHeight:cropHeight+SlidingWinSize,cropWidth+SlidingWinSize:cropWidth+SlidingWinSize+5]=sari
    return ImageDraw

