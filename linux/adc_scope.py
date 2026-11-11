import argparse
import json
import math
import struct
import subprocess
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

ADS, AD = b"ADSDW41!", b"V32CAP3!"


def frames(port):
    buf = bytearray()
    while chunk := port.read(4096):
        buf.extend(chunk)
        while True:
            starts = [pos for magic in (ADS, AD) if (pos := buf.find(magic)) >= 0]
            if not starts:
                del buf[:-7]
                break
            del buf[:min(starts)]
            ads = buf.startswith(ADS)
            header, stride = (24, 12) if ads else (40, 4)
            if len(buf) < header:
                break
            if ads:
                fid, fs, index, points = struct.unpack_from("<IIII", buf, 8)
                dec = 1
                valid = points == 32 and fs > 0
            else:
                ver, fid, fs, dec, points, _, index = struct.unpack_from("<IIIIIIQ", buf, 8)
                valid = ver == 3 and fs > 0 and 1 <= dec <= 32768 and 1 <= points <= 8192
            if not valid:
                del buf[:1]
                continue
            end = header + points * stride
            if len(buf) < end + 4:
                break
            if zlib.crc32(buf[:end]) != struct.unpack_from("<I", buf, end)[0]:
                del buf[:1]
                continue
            payload = buf[header:end]
            del buf[:end + 4]
            if ads:
                values = [int.from_bytes(payload[i:i+3], "little", signed=True)
                          for i in range(0, len(payload), 3)]
                channels = [values[i::4] for i in range(4)]
            else:
                channels = [list(c) for c in zip(*struct.iter_unpack("<hh", payload))]
            yield dict(source="ADS131M04" if ads else "AD9238", id=fid,
                       index=index, fs=fs / dec, channels=channels)


def measurements(channels, fs, frequency):
    count = len(channels[0])
    basis = None
    if math.isfinite(frequency) and 0 < frequency < fs / 2 and (count-1)*frequency >= 2*fs:
        angles = [2*math.pi*frequency*i/fs for i in range(count)]
        c = [math.cos(t) for t in angles]
        s = [math.sin(t) for t in angles]
        cm, sm = sum(c)/count, sum(s)/count
        c, s = [v-cm for v in c], [v-sm for v in s]
        cc, ss, cs = sum(v*v for v in c), sum(v*v for v in s), sum(a*b for a,b in zip(c,s))
        det = cc*ss-cs*cs
        if det > 1e-12:
            basis = c,s,cc,ss,cs,det
    rows = []
    for a in channels:
        mean = sum(a)/count
        centered = [v-mean for v in a]
        power = sum(v*v for v in centered)
        row = dict(dc=mean, vpp=max(a)-min(a), rms=math.sqrt(power/count))
        if basis and power > 0:
            c,s,cc,ss,cs,det = basis
            yc, ys = sum(v*w for v,w in zip(centered,c)), sum(v*w for v,w in zip(centered,s))
            x, y = (yc*ss-ys*cs)/det, (ys*cc-yc*cs)/det
            residual = max(0, power-x*yc-y*ys)
            row["residual_rms"] = math.sqrt(residual/count)
            if residual <= 0.1*power:
                row.update(peak=math.hypot(x,y), phase=math.degrees(math.atan2(-y,x)))
        rows.append(row)
    ref = rows[0]
    if ref.get("peak", 0) > 0:
        for row in rows:
            if "peak" in row:
                row["ratio"] = row["peak"]/ref["peak"]
                row["relative_phase"] = (row["phase"]-ref["phase"]+180)%360-180
    return rows


def load_calibration(path):
    with open(path, encoding="utf-8") as stream:
        rows = json.load(stream)
    if not isinstance(rows, list) or len(rows) != 4:
        raise ValueError("calibration requires four channels")
    result = [(float(r["offset_counts"]), float(r["volts_per_count"])) for r in rows]
    if any(not math.isfinite(o) or abs(o)>8388608 or not math.isfinite(k)
           or not 0 < abs(k) <= 1 for o,k in result):
        raise ValueError("invalid calibration coefficient")
    return result


HTML = """<!doctype html><meta charset=utf-8>
<title>ADC 波形</title>
<style>
body{background:#101820;color:#eee;font:20px sans-serif;margin:20px}
canvas{width:100%;height:58vh;background:#081017}
td,th{padding:2px 8px;text-align:right}table{font-size:15px}
</style>
<label>ADS 时基 <select id="seconds"><option>0.1</option><option selected>1</option><option>5</option><option>10</option></select> 秒</label>
<label>波形 <select id="coupling"><option>DC</option><option>AC</option></select></label>
<label>ADS 拟合频率 <input id="frequency" type="number" min="0" step="any" placeholder="Hz"></label>
<p id="status">等待串口帧</p><canvas width="1200" height="600"></canvas>
<table><thead><tr><th>通道</th><th>均值</th><th>AC RMS</th><th>峰峰值</th><th>拟合峰值</th><th>残差 RMS</th><th>幅值比</th><th>相对 CH0 相位</th></tr></thead><tbody id="stats"></tbody></table>
<script>
const canvas=document.querySelector('canvas'), ctx=canvas.getContext('2d');
const statusLine=document.getElementById('status'), colors=['#60defe','#ffd15c','#f683ce','#a9ee78'];
const stats=document.getElementById('stats'), fmt=v=>v===undefined?'—':v.toPrecision(4);
async function update(){
 try {
  const query=new URLSearchParams({seconds:document.getElementById('seconds').value,
    frequency:document.getElementById('frequency').value});
  const r=await (await fetch('/data?'+query,{cache:'no-store'})).json();
  ctx.clearRect(0,0,1200,600);
  stats.replaceChildren();
  if(!r.channels){statusLine.textContent=r.status;return;}
  const n=r.channels[0].length, ms=1000*(n-1)/r.fs;
  statusLine.textContent=r.source+' · 帧 '+r.id+' · '+r.fs.toFixed(2)+' Sa/s · '+ms.toFixed(3)+' ms · '+(r.unit||'ADC 码')+' · 各通道独立纵轴';
  const ac=document.getElementById('coupling').value==='AC';
  const h=600/r.channels.length;
  r.channels.forEach((samples,k)=>{
   const m=r.measurements?.[k];
   const a=ac&&m?samples.map(v=>v-m.dc):samples;
   const lo=Math.min(...a), hi=Math.max(...a), span=hi-lo;
   if(m){
    const row=stats.insertRow();
    ['CH'+k,...[m.dc,m.rms,m.vpp,m.peak,m.residual_rms,m.ratio,m.relative_phase].map(fmt)]
      .forEach(value=>row.insertCell().textContent=value);
   }
   ctx.strokeStyle=colors[k];ctx.fillStyle=colors[k];
   ctx.font='20px sans-serif';
   ctx.fillText('CH'+k+'   '+fmt(lo)+' … '+fmt(hi),12,k*h+24);
   ctx.beginPath();
   a.forEach((v,i)=>{
    const x=12+i*1176/Math.max(1,n-1), y=(k+1)*h-10-(span?(v-lo)/span:0.5)*(h-46);
    if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);
   });
   ctx.stroke();
  });
 }catch(e){statusLine.textContent='接收服务不可用';stats.replaceChildren();ctx.clearRect(0,0,1200,600);}
 finally{setTimeout(update,250);}
}
update();
</script>"""


def serve(device, baud, http_port, calibration=None):
    subprocess.run(["stty", "-F", device, str(baud), "raw", "-echo",
                    "cs8", "-cstopb", "-parenb", "-crtscts", "clocal", "min", "1", "time", "0"], check=True)
    latest = None
    history = None

    def receive():
        nonlocal latest, history
        with open(device, "rb", buffering=0) as stream:
            for frame in frames(stream):
                if frame["source"] == "ADS131M04":
                    count = len(frame["channels"][0])
                    if (history and history["fs"] == frame["fs"]
                            and history["next"] == frame["index"]
                            and history["id"] == (frame["id"] - 1) % (1 << 32)):
                        frame["channels"] = [(a + b)[-max(count, int(frame["fs"] * 10)):] for a, b in
                                             zip(history["channels"], frame["channels"])]
                    history = dict(frame, next=(frame["index"] + count) % (1 << 32))
                else:
                    history = None
                latest = (time.monotonic(), frame)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                body, mime = HTML.encode(), "text/html; charset=utf-8"
            elif urlsplit(self.path).path == "/data":
                snapshot = latest
                data = (snapshot[1] if snapshot and time.monotonic()-snapshot[0] < 2
                        else {"status": "等待有效串口帧"})
                if data.get("source") == "ADS131M04":
                    query = parse_qs(urlsplit(self.path).query)
                    value = query.get("seconds", ["1"])[0]
                    seconds = float(value) if value in {"0.1", "1", "5", "10"} else 1
                    limit = max(2, int(data["fs"] * seconds))
                    channels = [c[-limit:] for c in data["channels"]]
                    if calibration:
                        channels = [[(v-offset)*scale for v in channel]
                                    for channel,(offset,scale) in zip(channels,calibration)]
                    try:
                        frequency = float(query.get("frequency", ["0"])[0])
                    except ValueError:
                        frequency = 0
                    data = dict(data, channels=channels, unit="V" if calibration else "ADC 码",
                                measurements=measurements(channels,data["fs"],frequency))
                body, mime = json.dumps(data, ensure_ascii=False).encode(), "application/json; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    threading.Thread(target=receive, daemon=True).start()
    print(f"http://127.0.0.1:{http_port}/", flush=True)
    ThreadingHTTPServer(("127.0.0.1", http_port), Handler).serve_forever()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("device")
    parser.add_argument("--baud", type=int, default=2000000)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--calibration", help="four-channel calibration JSON")
    args = parser.parse_args()
    serve(args.device, args.baud, args.port,
          load_calibration(args.calibration) if args.calibration else None)

