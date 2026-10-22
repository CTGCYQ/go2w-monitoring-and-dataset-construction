"""Headless browser acceptance with synthetic point clouds, never robot commands."""
import argparse
import json
import math
import sys
import time
from pathlib import Path
from urllib.parse import urlparse

ROOT=Path(__file__).resolve().parents[1]


def make_room():
    points=[]
    for a in range(111):
        x=-3.5+a*.075
        for b in range(81):
            y=-3+b*.075
            points.append([x,y,-.65,.3])
    for i in range(101):
        a=-3.5+i*.075
        for j in range(45):
            z=-.65+j*.065
            points.extend([[a,3,z,.8],[4,a*.8,z,.7]])
    for i in range(70):
        y=-1.8+i*.045
        for j in range(24):points.append([1.4,y,-.65+j*.04,.6])
    for i in range(45):
        x=.5+i*.04
        for j in range(72):points.append([x,-1.8+j*.045,.31,.65])
    for i in range(120):
        a=i*math.tau/120
        for j in range(26):points.append([-1.5+.35*math.cos(a),1+.35*math.sin(a),-.65+j*.04,.75])
    return points


def main():
    p=argparse.ArgumentParser();p.add_argument('--chrome',default=r'C:\Program Files\Google\Chrome\Application\chrome.exe')
    p.add_argument('--output',default='tmp/pcviz_test');p.add_argument('--html');args=p.parse_args()
    from playwright.sync_api import sync_playwright
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    room=make_room();control={'online':True,'moving':False,'sequence':0,'calls':0,'bad_map':False,'aligned':False}
    errors=[];checks=[]
    def check(name,condition):
        if not condition:
            print(json.dumps({'failed':name,'errors':errors,'state':page.evaluate("({active:pcStudio.active, mode:pcStudio.buffer.mode,frames:pcStudio.buffer.frames.length,count:pcStudio.geometry.drawRange.count,info:pcStudio.renderer.info.render,camera:pcStudio.camera.position.toArray(),target:pcStudio.controls.target.toArray(),canvas:[pcStudio.renderer.domElement.width,pcStudio.renderer.domElement.height]})")},indent=2))
            page.locator('#pcStudio').screenshot(path=str(out/'debug.png'))
        assert condition,name
        checks.append(name)
    with sync_playwright() as pw:
        browser=pw.chromium.launch(executable_path=args.chrome,headless=True,args=['--enable-unsafe-swiftshader'])
        context=browser.new_context(viewport={'width':1500,'height':1100},device_scale_factor=1)
        page=context.new_page();page.on('pageerror',lambda e:errors.append(str(e)))
        page.on('console',lambda m:errors.append(m.text) if m.type=='error' and ('WebGL' in m.text or 'shader' in m.text.lower()) else None)
        def route(r):
            path=urlparse(r.request.url).path
            if path=='/api/lidar/cloud':
                control['calls']+=1;control['sequence']+=1
                data={'online':control['online'],'simulated':True,'frame_id':'utlidar_lidar','frame_key':str(control['sequence']),
                      'age_s':.03,'points':room[control['sequence']%6::6] if control['online'] else [],
                      'total':len(room)//6,'has_intensity':True,'motion':{'available':True,'moving':control['moving']}}
                if control['aligned']:data.update(display_direction_verified=True,display_alignment_id='synthetic',display_rotation_row_major=[0,-1,0,1,0,0,0,0,1])
                r.fulfill(json=data);return
            if path=='/api/map':
                if control['bad_map']:
                    r.fulfill(json={'online':False,'points':[],'reason_code':'map_geometry_invalid'});return
                r.fulfill(json={'online':False,'anchored':True,'written_at':time.time(),'points':room,'trajectory':[[i*.1,math.sin(i*.1),0] for i in range(20)],'count':len(room),'voxel':.1});return
            if path.startswith('/api/'):
                r.fulfill(json={'online':False,'power':{},'imu':{},'motor':[],'battery':{},'foot_force':[],'sport_mode':{}});return
            if path=='/' and args.html:
                r.fulfill(path=str(Path(args.html).resolve()),content_type='text/html; charset=utf-8');return
            rel=path.removeprefix('/static/') if path.startswith('/static/') else 'index.html' if path=='/' else path.lstrip('/')
            target=(ROOT/'web'/rel).resolve()
            if not target.is_relative_to((ROOT/'web').resolve()) or not target.is_file():r.fulfill(status=404,body='not found');return
            mime='application/javascript' if target.suffix=='.js' else 'text/css' if target.suffix=='.css' else 'text/html'
            r.fulfill(path=str(target),content_type=mime+'; charset=utf-8')
        page.route('https://pcviz.test/**',route)
        page.goto('https://pcviz.test/',wait_until='networkidle')
        page.locator('.tab[data-tab="vision"]').click();page.wait_for_function('typeof pcStudio!==\'undefined\' && pcStudio.geometry && pcStudio.geometry.drawRange.count > 1000')
        page.wait_for_function('pcStudio.renderer.info.render.points>1000',timeout=15000)
        check('native Z-up camera',page.evaluate('pcStudio.camera.up.z===1 && pcStudio.camera.up.y===0'))
        check('renderer has actual draw data',page.evaluate('pcStudio.renderer.info.render.points>1000'))
        check('no fake camera RGB mode',page.locator('#pcColor option[value="camera"]').count()==0)
        page.locator('#pcStudio').screenshot(path=str(out/'realtime.png'))
        page.locator('#pcFullscreen').click();page.wait_for_function('document.fullscreenElement!==null')
        check('fullscreen enabled',page.evaluate('document.fullscreenElement.id===\'pcStudio\''))
        page.locator('#pcFullscreen').click();page.wait_for_function('document.fullscreenElement===null')
        page.locator('[data-pc-mode="aggregate"]').click();page.wait_for_function('pcStudio.buffer.frames.length>=5',timeout=15000)
        check('aggregation receives distinct frames',page.evaluate('pcStudio.buffer.frames.length>=5'))
        check('aggregation builds detail',page.evaluate('pcStudio.buffer.cells.size>9000'))
        page.locator('#pcStudio').screenshot(path=str(out/'aggregate.png'))
        page.locator('[data-pc-view="top"]').click();page.wait_for_timeout(300)
        check('top preset preserves Z-up',page.evaluate('pcStudio.camera.position.z>pcStudio.controls.target.z'))
        page.locator('#pcStudio').screenshot(path=str(out/'top.png'))
        for mode in ['intensity','range','age','height']:page.select_option('#pcColor',mode)
        check('point colors finite',page.evaluate('Array.from(pcStudio.geometry.attributes.aColor.array.slice(0,pcStudio.geometry.drawRange.count*3)).every(Number.isFinite)'))
        page.locator('#pcPause').click();page.wait_for_timeout(300);before=control['calls'];page.wait_for_timeout(700)
        check('pause stops requests',control['calls']==before)
        page.locator('#pcPause').click();page.wait_for_timeout(500)
        control['moving']=True;page.wait_for_function('pcStudio.buffer.moving===true && pcStudio.buffer.frames.length<=1',timeout=10000)
        check('motion clears old aggregation',page.evaluate('pcStudio.buffer.frames.length<=1'))
        control['moving']=False;page.wait_for_timeout(600)
        control['online']=False;page.wait_for_function('pcStudio.geometry.drawRange.count===0',timeout=10000)
        check('offline clears live drawing',page.evaluate('pcStudio.geometry.drawRange.count===0'))
        check('offline state is explicit',page.locator('#pcLive').get_attribute('data-state')=='offline')
        page.locator('#pcStudio').screenshot(path=str(out/'offline.png'))
        control['online']=True;page.wait_for_function('pcStudio.geometry.drawRange.count>0',timeout=10000)
        check('automatic reconnect',page.evaluate('pcStudio.geometry.drawRange.count>0'))
        control['aligned']=True;page.wait_for_function('pcStudio.lastPacket.display_direction_verified===true',timeout=10000)
        check('calibrated direction shown explicitly',page.evaluate('document.getElementById("pcSourceNote").textContent.includes("前向已标定")'))
        check('display alignment reaches renderer',page.evaluate('pcStudio.buffer.frameId.endsWith(":synthetic") && pcStudio.geometry.drawRange.count>0'))
        page.locator('.tab[data-tab="overview"]').count()  # Discover first tab instead of assuming its name.
        page.locator('.tab').first.click();page.wait_for_timeout(400);before=control['calls'];page.wait_for_timeout(500)
        check('hidden tab stops acquisition',control['calls']==before)
        page.locator('.tab[data-tab="vision"]').click();page.wait_for_timeout(700)
        page.locator('[data-pc-mode="map"]').click();page.wait_for_function('pcStudio.buffer.mode==="map" && pcStudio.geometry.drawRange.count>1000')
        check('legacy world map remains available',page.evaluate('pcStudio.pathGeometry.drawRange.count>0'))
        check('offline map is clearly historical',page.locator('#pcLive').get_attribute('data-state')=='paused' and page.evaluate('pcStudio.lastPacket.history===true'))
        check('map has no fabricated per-point age',page.locator('#pcColor option[value="age"]').is_disabled())
        control['bad_map']=True;page.wait_for_function('pcStudio.geometry.drawRange.count===0',timeout=10000)
        check('corrupt map is blocked explicitly',page.evaluate('pcStudio.geometry.drawRange.count===0') and page.locator('#pcLive').get_attribute('data-state')=='offline')
        page.locator('[data-pc-mode="aggregate"]').click();page.wait_for_function('pcStudio.buffer.frames.length>=2')
        page.set_viewport_size({'width':390,'height':900});page.wait_for_timeout(500)
        check('mobile has no page overflow',page.evaluate('document.documentElement.scrollWidth<=window.innerWidth+2'))
        check('mobile renderer nonzero',page.evaluate('pcStudio.renderer.domElement.width>0'))
        page.evaluate('window.scrollTo(0,0)');page.wait_for_timeout(200)
        page.screenshot(path=str(out/'mobile.png'),full_page=True)
        check('no browser or shader errors',not errors)
        report={'passed':True,'checks':checks,'browser_errors':errors,'synthetic_test_data':True,'screenshots_saved':True,'real_sensor_data_saved':False,'fixture_point_count':len(room)}
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(report,indent=2));context.close();browser.close()

if __name__=='__main__':main()
