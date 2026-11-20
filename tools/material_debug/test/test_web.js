'use strict';
const assert = require('node:assert/strict');
const {observationView,point,DisplayRate} = require('../web/app.js');
const bbox = {color_id:1,color_name:'red',raw_label:'red_top',status:'BBOX_ONLY',valid:true,
  yolo_confidence:.9,bbox_center_px:[20,30],circle_center_px:null,center_px:[20,30],
  center_source:'bbox',geometry_reason:'NO_CIRCLE',conflicting_colors:[]};
assert.equal(point(null),'—');
assert.equal(point([123.456,78.9]),'(123.46, 78.90)');
assert.equal(observationView(bbox).circle,'未取得');
assert.equal(observationView(bbox).final,'(20.00, 30.00)');
assert.match(observationView(bbox).status,/可用/);
assert.equal(observationView({...bbox,geometry_reason:'NOT_APPLICABLE'}).circle,'不适用');
assert.equal(observationView({...bbox,circle_center_px:[23,34]}).delta,'5.00');
assert.equal(observationView({...bbox,yolo_confidence:null}).confidence,'—');
assert.match(observationView({...bbox,status:'COLOR_REJECTED'}).status,/拒判/);
assert.equal(observationView({...bbox,color_confidence:.95,color_d2:2.123}).colorConfidence,'95.00%');
assert.equal(observationView({...bbox,color_confidence:.95,color_d2:2.123}).colorDistance,'2.12');
const rate = new DisplayRate();
assert.equal(rate.update('1',1000),'0.0');
assert.equal(rate.update('2',1100),'10.0');
assert.equal(rate.update('2',1200),'10.0');
assert.equal(rate.update('2',2300),'0.0');

const vm = require('node:vm'), fs = require('node:fs');
class Element {
  constructor() { this.children=[]; this.hidden=false; this.textContent=''; }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children=[...children]; }
  removeAttribute(name) { delete this[name]; }
  addEventListener(name,fn) { this[name]=fn; }
  async decode() {}
}
async function pageTransitions() {
  const elements=new Map(), callbacks=[];
  const byId=id => { if (!elements.has(id)) elements.set(id,new Element()); return elements.get(id); };
  const payload={state:'READY',message:'识别中',source:'camera',backend:'yolo',model_kind:'top',model_path:'/tmp/model.bin',class_labels:['red_top'],confidence_threshold:.5,
    image_base64:'JPEG',display_frame_seq:1,camera_frame_seq:1,image_size:[1280,720],camera_fps:30,processing_fps:15,result_age_ms:60,device:'/dev/test',
    result:{processing_ms:50,stages_ms:{yolo:30,geometry:20},observations:[bbox,{...bbox,bbox_center_px:[100,100],center_px:[100,100]}]}};
  let current=payload, postRequest=null, postError=null;
  vm.runInNewContext(fs.readFileSync(require.resolve('../web/app.js'),'utf8'),{
    document:{getElementById:byId,createElement:()=>new Element()},
    fetch:async(url,options)=>{
      if (url==='/api/mode') {
        postRequest=JSON.parse(options.body);
        return {ok:!postError,json:async()=>postError ? {error:postError} : {accepted:true}};
      }
      if (current instanceof Error) throw current; return {ok:true,json:async()=>current};
    },
    AbortController,performance:{now:()=>1000},
    setTimeout:(fn,delay)=>{ if (delay===10) callbacks.push(fn); return 1; },clearTimeout(){}
  });
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(byId('preview').hidden,false);
  assert.equal(byId('observations').children.length,2); // same color, two instances
  assert.equal(byId('camera-fps').textContent,'30.0');
  assert.match(byId('rule').textContent,/0.5/);
  byId('model-select').value='jhb';
  await byId('model-select').change();
  assert.equal(postRequest.use_jhb_model,true);
  assert.equal(byId('observations').children.length,0);
  current={...payload,state:'SWITCHING',message:'正在加载',switching:true,result:null,
    mode_revision:1};
  await callbacks.shift()();
  assert.equal(byId('model-select').disabled,true);
  assert.match(byId('switch-status').textContent,/加载/);
  current={...payload,backend:'jhb',model_kind:'jhb',refine_circle:true,use_jhb_model:true,mode_revision:1,
    class_labels:['material','cap_top'],
    result:{...payload.result,rejected_candidates:[{reason:'CAP_UNPAIRED'}],
      observations:[{...bbox,identity_source:'jhb_lab',color_confidence:.95,color_d2:2.12,color_reason:'OK'}]}};
  await callbacks.shift()();
  assert.match(byId('mode').textContent,/JHB.*Lab/);
  assert.equal(byId('model-select').value,'jhb');
  assert.equal(byId('model-select').disabled,false);
  assert.match(byId('rule').textContent,/筛除顶帽 1 个/);
  assert.equal(byId('observations').children.length,1);
  assert.ok(byId('observations').children[0].children[2].children.some(e=>e.textContent==='Lab 判色置信度'));
  byId('model-select').value='test_combine';
  await byId('model-select').change();
  assert.deepEqual(postRequest,{use_jhb_model:false,test_combine:true});
  current={...payload,backend:'test_combine',model_kind:'test_combine',test_combine:true,
    color_model_path:'/old/yolo.bin',
    result:{...payload.result,observations:[{...bbox,identity_source:'yolo_jhb',
      color_confidence:.95,color_bbox_xyxy:[10,20,30,40],color_reason:'OK'}]}};
  await callbacks.shift()();
  assert.equal(byId('model-select').value,'test_combine');
  assert.match(byId('mode').textContent,/不运行 Lab/);
  assert.match(byId('model').textContent,/颜色模型：\/old\/yolo.bin/);
  assert.ok(byId('observations').children[0].children[2].children.some(e=>e.textContent==='原 YOLO 颜色置信度'));
  current={...current,test_combine:false,use_jhb_model:true,backend:'jhb'};
  postError='正在切换模型';
  byId('model-select').value='original';
  await byId('model-select').change();
  await callbacks.shift()();
  assert.match(byId('switch-status').textContent,/请求失败.*正在切换/);
  assert.equal(byId('model-select').value,'jhb');
  current={...payload,state:'STALE_RESULT',result:null,image_base64:null,display_frame_seq:null};
  await callbacks.shift()();
  assert.equal(byId('observations').children.length,0);
  assert.equal(byId('preview').hidden,true);
  current=new Error('offline');
  await callbacks.shift()();
  assert.equal(byId('state').textContent,'连接中断');
  assert.equal(byId('observations').children.length,0);
  assert.equal(byId('camera-fps').textContent,'—');
  console.log('material web tests passed');
}
pageTransitions().catch(error=>{console.error(error);process.exitCode=1;});
