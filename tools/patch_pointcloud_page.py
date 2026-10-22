"""Upgrade only the point-cloud region in an existing monitor HTML page."""
import argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def upgrade(source,template):
    a=template.index('<section class="card pc-studio"');b=template.index('</section>',a)+len('</section>')
    panel=template[a:b]
    a=source.index('id="tv-vision"');a=source.index('<div class="card"',a)
    b=source.index('<div class="grid g3">',a)
    source=source[:a]+panel+'\n    '+source[b:]
    a=source.rfind('// =================',0,source.index('let pcFwdArrow'))
    b=source.index('let agentCamTimer',a)
    begin=template.index('// Point-cloud module owns')
    end=template.index('let agentCamTimer',begin)
    source=source[:a]+template[begin:end]+source[b:]
    source=source.replace('switchPcViewMode();',"if(document.getElementById('tv-vision').classList.contains('active')) pcStudio?.resume();")
    marker='if(wmActive && wmRenderer)'
    if marker in source:
        a=source.index(marker);brace=source.index('{',a);depth=1;i=brace+1
        while depth:
            if source[i]=='{':depth+=1
            if source[i]=='}':depth-=1
            i+=1
        source=source[:a]+source[i:]
    source=source.replace('else if(renderer){','if(renderer){')
    hook="  if (name==='agent') { startAgentView(); } else { stopAgentView(); }"
    source=source.replace(hook,"  if(name!=='vision') pcStudio?.suspend();\n"+hook)
    source=source.replace('    renderer.setSize(el.clientWidth, el.clientHeight);\n    camera.aspect = el.clientWidth/el.clientHeight;\n    camera.updateProjectionMatrix();','    pcStudio?.resize();')
    assets='<link rel="stylesheet" href="/static/pointcloud-studio.css?v=20261022-5">\n<script src="/static/pointcloud-core.js?v=20261022-5"></script>\n<script src="/static/pointcloud-studio.js?v=20261022-5"></script>\n'
    source=source.replace('</head>',assets+'</head>')
    return source


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();source=Path(args.input).read_text(encoding='utf-8');template=(ROOT/'web/index.html').read_text(encoding='utf-8')
    Path(args.output).write_text(upgrade(source,template),encoding='utf-8')

if __name__=='__main__':main()
