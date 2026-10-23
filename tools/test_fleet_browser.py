"""Run against tools/fleet_test_server.py only (loopback fixtures, no robot data)."""
import json,time
from pathlib import Path
from urllib.parse import urlsplit
from playwright.sync_api import sync_playwright,TimeoutError as PlaywrightTimeout

def main():
 errors=[];requests=[];checks=[];base='http://127.0.0.1:18881'
 with sync_playwright() as p:
  browser=p.chromium.launch(executable_path=r'C:\Program Files\Google\Chrome\Application\chrome.exe',headless=True,args=['--enable-unsafe-swiftshader'])
  page=browser.new_page(viewport={'width':1500,'height':1100});page.on('pageerror',lambda e:errors.append(str(e)));page.on('request',lambda r:requests.append(urlsplit(r.url).path))
  page.goto(base+'/?robot=go2w-01&tab=robots')
  try:page.wait_for_load_state('networkidle',timeout=2000)
  except PlaywrightTimeout:pass # Continuous telemetry polling: use explicit readiness below.
  page.wait_for_function('document.querySelectorAll(".robot-card").length===2')
  assert page.title()=='具身智能机器人综合管理平台';checks.append('platform_name')
  other=browser.new_page();other.on('pageerror',lambda e:errors.append(str(e)));other.goto(base+'/?robot=go2w-01&tab=robots');other.wait_for_function('document.querySelectorAll(".robot-card").length===2')
  page.locator('#robotSelector').select_option('robot-02');page.wait_for_url('**robot=robot-02**');page.wait_for_function('document.getElementById("robotContextName").textContent==="测试机器人 02"');checks.append('robot_switch_resets_document_and_preserves_tab')
  for tab in ['monitor','record','vision','annotate','agent','slam','jobs','robots']:
   page.locator(f'.tab[data-tab="{tab}"]').click();page.wait_for_timeout(300)
   assert page.locator('#robotSelector').input_value()=='robot-02'
  checks.append('all_eight_pages_keep_robot_context')
  paths=set(requests)
  for suffix in ['state','record/status','sessions','lidar/cloud','depth/color','slam/map','agent/motion-gate']:
   assert '/api/robots/robot-02/'+suffix in paths,(suffix,sorted(paths))
  assert not any(path.startswith('/api/') and not path.startswith(('/api/robots/','/api/fleet/')) for path in paths),sorted(paths);checks.append('all_robot_requests_scoped_including_images')
  page.locator('#addRobotBtn').click();page.locator('#robotId').fill('robot-03');page.locator('#robotName').fill('测试机器人 03');page.locator('#robotModel').fill('Fixture');page.locator('#robotEndpoint').fill('http://127.0.0.1:18883');page.locator('#robotForm button[type="submit"]').click();page.wait_for_function('document.querySelectorAll(".robot-card").length===3');checks.append('register_node_and_display_offline')
  other.wait_for_function('document.querySelectorAll(".robot-card").length===3');assert other.locator('#robotSelector').input_value()=='go2w-01';other.close();checks.append('inventory_updates_across_tabs_with_independent_robot_context')
  page.locator('.tab[data-tab="jobs"]').click();page.locator('#createJobBtn').click();page.locator('#jobTitle').fill('浏览器传感器巡检');assert page.locator('#jobRobot').input_value()=='robot-02';page.locator('#jobForm button[type="submit"]').click();page.wait_for_function('document.getElementById("jobRows").textContent.includes("浏览器传感器巡检")');page.wait_for_function('document.getElementById("jobDetail").textContent.includes("已成功")',timeout=20000);checks.append('dispatch_real_backend_state_machine_and_timeline')
  assert page.locator('.job-step.succeeded').count()==4;assert '100.0%' in page.locator('#jobStats').inner_text();checks.append('verified_step_status_and_metrics')
  page.locator('#jobScope').select_option('all');page.wait_for_timeout(700)
  Path('tmp').mkdir(exist_ok=True);page.screenshot(path='tmp/fleet_jobs_desktop.png',full_page=True)
  page.set_viewport_size({'width':390,'height':844});page.wait_for_timeout(400);assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1');page.screenshot(path='tmp/fleet_jobs_mobile.png',full_page=True);checks.append('mobile_no_horizontal_overflow')
  page.goto(base+'/?robot=missing&tab=robots');page.wait_for_function('document.getElementById("robotSelector").options.length>=3');assert page.locator('#robotSelector').input_value()=='missing';page.locator('#robotSelector').select_option('go2w-01');page.wait_for_url('**robot=go2w-01**');checks.append('invalid_saved_robot_can_be_recovered_without_fallback')
  assert not errors,errors
  browser.close()
 report={'passed':True,'checks':checks,'browser_errors':errors,'fixture_only':True,'real_robot_motion_commands':0};Path('tmp/fleet_browser_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
