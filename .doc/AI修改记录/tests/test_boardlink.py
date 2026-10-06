#!/usr/bin/env python3
"""Execute the actual protocol C on the host; hardware is replaced only at HAL boundaries."""
import ctypes as C, pathlib, subprocess, tempfile, struct, math, os, shlex
ROOT=pathlib.Path(__file__).resolve().parents[4]
TESTS=pathlib.Path(__file__).resolve().parent

def library(tmp,board,transport):
    path=pathlib.Path(tmp)/f'{board}_{transport}.so'
    subprocess.run([os.environ.get('CC','cc'),*shlex.split(os.environ.get('HOST_CFLAGS','')),'-shared','-fPIC','-std=c11','-Wall','-Wextra',f'-DBOARD_LINK_TRANSPORT={transport}',
        '-I'+str(TESTS/'stubs'),'-I'+str(ROOT/f'27_Infantry_{board}/User/System'),'-I'+str(ROOT/f'27_Infantry_{board}/User/Software/Infantry'),
        str(ROOT/f'27_Infantry_{board}/User/Software/Infantry/BoardLink.c'),str(TESTS/'link_harness.c'),'-o',str(path)],check=True)
    lib=C.CDLL(str(path));lib.Test_Get.restype=C.c_float
    lib.Test_Time.argtypes=[C.c_uint32]
    lib.BoardLink_Init();return lib

def buf(data): return (C.c_uint8*len(data)).from_buffer_copy(data)
def packets(lib):
    result=[]
    for i in range(lib.Test_Count()):
        b=(C.c_uint8*128)(); n=lib.Test_Packet(i,b);result.append((lib.Test_Id(i),bytes(b[:n])))
    lib.Test_Clear();return result

def deliver(src,dst,transport,split=True):
    result=packets(src)
    for ident,data in result:
        if transport==1: assert dst.Test_Can(1,ident,buf(data))==1
        elif split:
            for byte in data: assert dst.Test_Serial(1 if transport==3 else 2,buf(bytes([byte])),1)==1
        else: dst.Test_Serial(1 if transport==3 else 2,buf(data),len(data))
    src.Test_Complete();return result

def crc(data):
    value=0xffff
    for b in data:
        value^=b<<8
        for _ in range(8): value=((value<<1)^0x1021 if value&0x8000 else value<<1)&0xffff
    return struct.pack('<H',value)
def frame(items):
    payload=b''.join(struct.pack('<HB',ident,len(data))+data for ident,data in items)
    data=b'\xa5\x5a\x02\x00'+bytes([len(items),len(payload)])+payload
    return data+crc(data)

def run(transport,tmp):
    g=library(tmp,'Gimbal',transport);c=library(tmp,'Chassis',transport)
    assert not c.BoardLink_CommandIsFresh()
    if transport==2:
        c.BoardLink_TxStep();assert c.Test_Count()==0, 'RS485 slave must wait for master'
    g.Test_Set();c.Test_Set()
    for tick in range(60):
        g.Test_Time(tick);c.Test_Time(tick)
        g.BoardLink_TxStep();deliver(g,c,transport)
        c.BoardLink_TxStep();deliver(c,g,transport)
    for k,want in enumerate([1.25,-2.5,.75,-3.2,2,2,4,1,1,37]):
        assert math.isclose(c.Test_Get(k),want,abs_tol=1e-5),(k,c.Test_Get(k),want)
    for k,want in {10:179.2,11:-1.75,12:24.5,13:77,14:240,15:333,16:12}.items():
        assert math.isclose(g.Test_Get(k),want,abs_tol=1e-4)
    assert c.BoardLink_CommandIsFresh() and g.BoardLink_BodyGyroIsFresh() and g.BoardLink_RelativeAngleIsFresh()
    c.Test_Time(81);g.Test_Time(81)
    assert not c.BoardLink_CommandIsFresh() and not g.BoardLink_RelativeAngleIsFresh()
    assert not g.BoardLink_BodyGyroIsFresh()
    def rx(ident,data):
        if transport==1:c.Test_Can(1,ident,buf(data))
        else:
            packet=frame([(ident,data)]);c.Test_Serial(1 if transport==3 else 2,buf(packet),len(packet))
    c.BoardLink_Init();c.Test_Time(100)
    rx(0x101,struct.pack('<ff',1,2));rx(0x102,struct.pack('<ff',0,3))
    rx(0x104,bytes([0,0,0,0,0,0,0,2]));rx(0x108,bytes([1,1,1,0,0,0,0,2]))
    assert c.BoardLink_CommandIsFresh()
    c.Test_Time(121);rx(0x102,struct.pack('<ff',0,3))
    assert not c.BoardLink_CommandIsFresh(), 'Yaw traffic must not keep stale movement/modes alive'
    for ident,data in [(0x101,struct.pack('<ff',float('nan'),2)),(0x102,struct.pack('<ff',0,float('inf'))),
                       (0x104,bytes([9,0,0,0,0,0,0,2])),(0x108,bytes([255,0,0,0,0,0,0,2]))]:
        c.BoardLink_Init();rx(ident,data);assert not c.BoardLink_CommandIsFresh()
    if transport==1:
        c.BoardLink_Init();assert c.Test_Can(0,0x101,buf(struct.pack('<ff',8,9)))==0
        assert c.Test_Get(0)==0
    else:
        port=1 if transport==3 else 2;c.BoardLink_Init()
        good=frame([(0x101,struct.pack('<ff',4,5))]);bad=good[:-1]+bytes([good[-1]^1])
        c.Test_Serial(port,buf(bad),len(bad));assert c.Test_Get(0)==0
        # Bad second TLV must not partially apply first valid TLV.
        malformed=bytearray(frame([(0x101,struct.pack('<ff',6,7)),(0x102,struct.pack('<ff',0,1))]))
        malformed[19]=7;malformed[-2:]=crc(malformed[:-2])
        c.Test_Serial(port,buf(malformed),len(malformed));assert c.Test_Get(0)==0
        noise=b'\x00\x99\xa5'+good+good
        c.Test_Serial(port,buf(noise),len(noise));assert c.Test_Get(0)==4
        g.BoardLink_Init();g.Test_Set();g.BoardLink_TxStep();assert g.Test_Count()==1
        g.Test_Clear();g.BoardLink_TxStep();assert g.Test_Count()==0, 'DMA busy buffer cannot be overwritten'
        g.Test_Complete();g.Test_Fail(1);g.Test_Time(1000);g.BoardLink_TxStep();assert g.Test_Count()==0
        g.Test_Fail(0);g.Test_Time(1005);g.BoardLink_TxStep();assert g.Test_Count()==1
    # Tick overflow: unsigned age must remain valid across wrap.
    c.BoardLink_Init();c.Test_Time(0xfffffff8)
    rx(0x101,struct.pack('<ff',1,2));rx(0x102,struct.pack('<ff',0,3))
    rx(0x104,bytes([0,0,0,0,0,0,0,2]));rx(0x108,bytes([1,1,1,0,0,0,0,2]))
    c.Test_Time(2);assert c.BoardLink_CommandIsFresh()
    c.Test_Time(20);assert not c.BoardLink_CommandIsFresh()
    print(f'PASS BoardLink transport {transport}: bidirectional C roundtrip, modes, freshness, invalid frames, transport isolation')
if __name__=='__main__':
    with tempfile.TemporaryDirectory() as tmp:
        for transport in [1,2,3]:run(transport,tmp)
