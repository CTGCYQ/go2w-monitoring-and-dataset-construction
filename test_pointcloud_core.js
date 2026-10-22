const assert=require('node:assert/strict');const {CloudBuffer,color}=require('./web/pointcloud-core.js');
let checks=0;function test(name,run){run();checks++;console.log('PASS '+name)}
const packet=(id,points,moving=false)=>({online:true,frame_id:'lidar',frame_key:String(id),points,motion:{moving}});
test('preserves actual right-handed source axes',()=>{const b=new CloudBuffer();b.ingest(packet(1,[[1,2,3,.8]]),1);const p=[...b.cells.values()][0];assert.deepEqual([p.x,p.y,p.z],[1,2,3]);});
test('ignores duplicate frames',()=>{const b=new CloudBuffer();b.mode='aggregate';b.ingest(packet(1,[[1,0,0]]),1);assert.equal(b.ingest(packet(1,[[2,0,0]]),2),false);assert.equal(b.frames.length,1)});
test('expires old voxels and frames',()=>{const b=new CloudBuffer();b.mode='aggregate';b.window=5;b.ingest(packet(1,[[1,0,0]]),1);b.ingest(packet(2,[[2,0,0]]),8);assert.equal(b.cells.size,1);assert.equal([...b.cells.values()][0].x,2)});
test('motion resets static accumulation',()=>{const b=new CloudBuffer();b.mode='aggregate';b.ingest(packet(1,[[1,0,0]]),1);b.ingest(packet(2,[[2,0,0]],true),2);assert.equal(b.cells.size,1);b.ingest(packet(3,[[3,0,0]]),3);assert.equal(b.cells.size,1)});
test('hard memory bound',()=>{const b=new CloudBuffer();b.mode='aggregate';b.limit=10;b.ingest(packet(1,Array.from({length:200},(_,i)=>[i*.04+1,0,0])),1);assert.equal(b.cells.size,10)});
test('drops invalid and out-of-range points',()=>{const b=new CloudBuffer();b.ingest(packet(1,[[NaN,0,0],[Infinity,0,0],[false,0,0],[99,0,0],[1,0,0]]),1);assert.equal(b.cells.size,1)});
test('source frame changes clear aggregation',()=>{const b=new CloudBuffer();b.mode='aggregate';b.ingest(packet(1,[[1,0,0]]),1);b.ingest({...packet(2,[[2,0,0]]),frame_id:'new'},2);assert.equal(b.cells.size,1)});
test('colors stay finite at extrema',()=>{for(const t of [-10,0,.5,1,10])assert(color(t).every(v=>Number.isFinite(v)&&v>=0&&v<=1))});
console.log(JSON.stringify({passed:true,checks}));
