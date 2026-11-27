"""Isolated synthetic ROS -> trained critic -> sampler -> MCAP -> loopback upload soak.
Requires ROS sourced and numpy/Pillow/rclpy; never reads real camera frames or device credentials.
"""
import argparse, hashlib, http.server, io, json, math, os, pathlib, signal, sqlite3
import subprocess, threading, time
import numpy as np
from PIL import Image
import rclpy
from rclpy.context import Context
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, CompressedImage
from std_msgs.msg import Float64MultiArray, String
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message

p=argparse.ArgumentParser()
for name in ('binary','transport_fixture','binding','selection','bundle','root','disk_shim'):
 p.add_argument('--'+name.replace('_','-'),required=True)
p.add_argument('--domain',type=int,default=197)
p.add_argument('--duration',type=float,default=130)
p.add_argument('--min-samples',type=int,default=3)
p.add_argument('--with-candidate',action='store_true',help='exercise candidate registration and the official export-window command')
p.add_argument('--candidate-at',type=float,default=125)
a=p.parse_args()
if a.with_candidate and a.duration<a.candidate_at+25:p.error("candidate check needs at least 25 seconds after --candidate-at")
root=pathlib.Path(a.root);root.mkdir(parents=True,exist_ok=False)
store=root/'store';sessions=root/'sessions';sessions.mkdir();session=sessions/'synthetic'
env={**os.environ,'REARGUARD_STATE_DIR':str(root/'events'),'NO_PROXY':'127.0.0.1','no_proxy':'127.0.0.1'}
binding=json.loads(pathlib.Path(a.binding).read_text());binding['clock']='ros_system'
(root/'binding.json').write_text(json.dumps(binding))
init=subprocess.run([a.binary,'workflow','init','--store',str(store),'--config',str(root/'binding.json'),
 '--profile-id','isolated-observer-soak'],env=env,capture_output=True,text=True)
if init.returncode:raise RuntimeError(init.stderr)
(store/'models').mkdir(exist_ok=True)
with sqlite3.connect(store/'workflow.sqlite3') as db:profile=json.loads(db.execute('SELECT body FROM profile WHERE id=1').fetchone()[0])
selection=json.loads(pathlib.Path(a.selection).read_text())
selection.update(profile_id=profile['profile_id'],binding_hash=profile['binding_hash'])
assert selection['mode']=='shadow' and not selection['enforcement_authorized']
(root/'selection.json').write_text(json.dumps(selection))
objects={};completed={};requests=[];lost_ack=False;mock_errors=[]
class Handler(http.server.BaseHTTPRequestHandler):
 protocol_version='HTTP/1.1'
 def log_message(self,*args):pass
 def reply(self,status,payload):
  body=json.dumps(payload).encode();self.send_response(status);self.send_header('Content-Type','application/json')
  self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
 def do_POST(self):
  global lost_ack
  try:
   data=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
   assert self.headers['X-Rearguard-Device-Secret']=='0'*64
   key=data['recording_id'];kind=data['manifest']['kind'];requests.append((self.path,kind,key))
   if self.path.endswith('/upload'):
    time.sleep(.15)
    self.reply(200,{'upload_url':f'http://127.0.0.1:{self.server.server_port}/objects/{key}',
      'required_headers':{'If-None-Match':'*'},'object_key':key,'expires_at':'2099-01-01T00:00:00Z'})
   elif key not in objects:self.reply(404,{'error':'object not uploaded'})
   elif kind=='execution_sample' and not lost_ack:
    lost_ack=True;self.reply(503,{'error':'injected lost completion acknowledgement'})
   else:
    assert objects[key]['sha256']==data['sha256'] and objects[key]['size']==data['size_bytes']
    completed[key]={'kind':kind,'sample_id':data['manifest'].get('sample_id'),'sha256':data['sha256']}
    self.reply(200,{})
  except Exception as e:
   mock_errors.append(str(e));self.reply(500,{'error':str(e)})
 def do_PUT(self):
  key=self.path.rsplit('/',1)[-1];size=int(self.headers['Content-Length']);remaining=size;digest=hashlib.sha256()
  with (root/('uploaded-'+key+'.mcap')).open('wb') as f:
   while remaining:
    block=self.rfile.read(min(1024*1024,remaining));assert block
    remaining-=len(block);digest.update(block);f.write(block)
  objects[key]={'sha256':digest.hexdigest(),'size':size};self.reply(200,{})
server=http.server.ThreadingHTTPServer(('127.0.0.1',0),Handler)
threading.Thread(target=server.serve_forever,daemon=True).start()
context=Context();rclpy.init(context=context,domain_id=a.domain)
node=rclpy.create_node('synthetic_observer_soak',context=context)
joint_pub=node.create_publisher(JointState,'/joint_states',qos_profile_sensor_data)
command_pub=node.create_publisher(Float64MultiArray,'/arm_controller/commands',qos_profile_sensor_data)
camera_topics=[t['name'] for t in binding['topics'] if t.get('role')=='camera' and t['name'].endswith('/compressed')]
assert len(camera_topics)==2
camera_pubs=[node.create_publisher(CompressedImage,t,qos_profile_sensor_data) for t in camera_topics]
rgb=np.random.default_rng(1984).integers(0,256,(480,640,3),dtype=np.uint8)
buf=io.BytesIO();Image.fromarray(rgb).save(buf,format='JPEG',quality=20);jpeg=buf.getvalue()
children=[];logs=[];states=[];published={'joints':0,'commands':0,'external':0,'wrist':0};heartbeat_samples=[]
try:
 for name,args,extra_env in [
  ('observer',[a.binary,'observe','start','--config',str(root/'binding.json'),'--session',str(session),
    '--store',str(store),'--domain-id',str(a.domain),'--wall-timeout',str(a.duration+45),
    '--physical-critic-bundle',a.bundle,'--physical-critic-selection',str(root/'selection.json')],{'LD_PRELOAD':a.disk_shim}),
  ('uploader',[a.transport_fixture,'--pipeline',str(store),str(sessions),f'http://127.0.0.1:{server.server_port}'],{})]:
  log=(root/(name+'.log')).open('w');logs.append(log)
  children.append(subprocess.Popen(args,env={**env,**extra_env},stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
 started=time.monotonic();next_joint=next_command=next_external=next_wrist=next_poll=0.;last_h=None
 candidate_pub=None;next_candidate_send=a.candidate_at;candidate_sent=False;candidate_export=None;export_log=None
 while (elapsed:=time.monotonic()-started)<a.duration:
  if children[0].poll() is not None:raise RuntimeError('observer exited: '+(root/'observer.log').read_text()[-1800:])
  stamp=node.get_clock().now().to_msg()
  active=any(s<=elapsed<s+10 for s in (15,55,95))
  value=.25*math.sin(elapsed*2) if active else 0.
  positions=[value]*len(binding['joint_order'])
  if elapsed>=next_joint:
   msg=JointState();msg.header.stamp=stamp;msg.name=binding['joint_order'];msg.position=positions
   joint_pub.publish(msg);published['joints']+=1;next_joint=elapsed+1/30
  if active and elapsed>=next_command:
   msg=Float64MultiArray();msg.data=positions;command_pub.publish(msg);published['commands']+=1;next_command=elapsed+.05
  for i,(pub,deadline) in enumerate(zip(camera_pubs,(next_external,next_wrist))):
   if elapsed>=deadline:
    msg=CompressedImage();msg.header.stamp=stamp;msg.format='jpeg';msg.data=jpeg;pub.publish(msg)
    published['external' if i==0 else 'wrist']+=1
    if i==0:next_external=elapsed+.16
    else:next_wrist=elapsed+.05
  if a.with_candidate and a.candidate_at<=elapsed<a.candidate_at+4 and elapsed>=next_candidate_send:
   if candidate_pub is None:
    prefix=json.loads((session/'session.json').read_text())['prefix']
    candidate_pub=node.create_publisher(String,prefix+'/requests',10)
   # Candidate admission requires a recent action event, too. This is a
   # synthetic publication on the isolated domain, never a hardware command.
   action=Float64MultiArray();action.data=positions;command_pub.publish(action);published['commands']+=1
   msg=String();msg.data=json.dumps({'schema_version':1,'request_id':'synthetic-soak-candidate',
    'operation':'candidate','detector_id':'synthetic-soak','evidence_class':'fixture',
    'confidence':.9,'before':10,'after':5})
   candidate_pub.publish(msg);candidate_sent=True;next_candidate_send=elapsed+1
  if elapsed>=next_poll:
   path=session/'state.json'
   if path.exists():
    state=json.loads(path.read_text());h=state.get('heartbeat_wall_ns')
    if h!=last_h:
     heartbeat_samples.append({'elapsed':elapsed,'queue':state.get('receive_queue'),
      'critic_status':state.get('critic',{}).get('status'),
      'inference_submissions':state.get('critic',{}).get('inference_submissions',0)})
     last_h=h;states.append(state)
   if a.with_candidate and candidate_sent and candidate_export is None:
    with sqlite3.connect(store/'workflow.sqlite3') as db:
     current_profile=json.loads(db.execute('SELECT body FROM profile WHERE id=1').fetchone()[0])
    for candidate_id,candidate in current_profile.get('candidates',{}).items():
     if candidate.get('window',{}).get('status')=='ready':
      export_log=(root/'candidate-export.log').open('w');logs.append(export_log)
      candidate_export=subprocess.Popen([a.binary,'workflow','export-window','--store',str(store),
       '--session',str(session),'--candidate',candidate_id],env=env,stdout=export_log,stderr=subprocess.STDOUT,start_new_session=True)
      children.append(candidate_export);break
   if int(elapsed)%15==0:
    print(json.dumps({'elapsed_s':round(elapsed),'published':published,'samples_submitted':len(list((store/'execution-samples').glob('*.submitted.json')))}),flush=True)
   next_poll=elapsed+1
  time.sleep(.002)
 (store/'test.stop').write_text('done')
 children[1].wait(timeout=30)
finally:
 for child in children:
  if child.poll() is None:
   os.killpg(child.pid,signal.SIGINT)
   try:child.wait(timeout=15)
   except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
 for log in logs:log.close()
 node.destroy_node();context.shutdown();server.shutdown();server.server_close()

spool=store/'execution-samples';submitted=list(spool.glob('*.submitted.json'));rejected=[]
for path in spool.glob('*.rejected.json'):rejected.append(json.loads(path.read_text()))
windows={}
for state in states:
 for w in state.get('windows',[]):
  if w.get('execution_sample') and w['status']=='ready':windows[w['id']]=w
max_gap=max((c.get('max_receipt_gap_ns',0)/1e6 for w in windows.values() for c in w['coverage'].values() if c.get('count',0)),default=None)
model_submissions=max((s['inference_submissions'] for s in heartbeat_samples),default=0)
queues=[s['queue'] for s in heartbeat_samples if s['queue']]
report={'binary':a.binary,'binary_sha256':hashlib.sha256(pathlib.Path(a.binary).read_bytes()).hexdigest(),
 'duration_s':a.duration,'synthetic_jpeg_bytes':len(jpeg),'published':published,
 'motion_episodes':sum(t<a.duration for t in (15,55,95)),'motion_seconds_each':10,'sample_windows':len(windows),'submitted_samples':len(submitted),
 'rejected_samples':rejected,'maximum_sample_receipt_gap_ms':max_gap,'model_inference_submissions':model_submissions,
 'valid_critic_heartbeats':sum(s['critic_status']=='valid' for s in heartbeat_samples),
 'peak_queue_bytes':max((q['peak_bytes'] for q in queues),default=None),
 'peak_queue_messages':max((q['peak_messages'] for q in queues),default=None),
 'queue_overflow':any(q['overflow'] for q in queues),'injected_completion_failure':lost_ack,
 'mock_completed_execution_samples':sum(x['kind']=='execution_sample' for x in completed.values()),
 'mock_completed_incidents':sum(x['kind']=='incident_evidence' for x in completed.values()),'mock_errors':mock_errors,'candidate_request_sent':candidate_sent,
 'official_candidate_export_returncode':candidate_export.returncode if candidate_export else None}
recording_gaps={};last_receipt={};recording_counts={}
recording_manifest=json.loads((session/'recording.json').read_text())
for segment in recording_manifest.get('segments',[]):
 reader=SequentialReader();reader.open(StorageOptions(uri=str(session/segment['path']),storage_id='mcap'),ConverterOptions('cdr','cdr'))
 while reader.has_next():
  topic,_,stamp=reader.read_next()
  if topic not in camera_topics+['/joint_states']:continue
  if topic in last_receipt:recording_gaps[topic]=max(recording_gaps.get(topic,0),stamp-last_receipt[topic])
  last_receipt[topic]=stamp;recording_counts[topic]=recording_counts.get(topic,0)+1
report['retained_recording_max_receipt_gaps_ms']={k:v/1e6 for k,v in recording_gaps.items()}
report['retained_recording_message_counts']=recording_counts
report['passed']=(len(submitted)>=a.min_samples and (a.min_samples==0 or not rejected)
 and model_submissions>100 and not report['queue_overflow'] and not mock_errors
 and (not a.with_candidate or report['mock_completed_incidents']>=1))
export_audit=[]
for marker in submitted:
 sample_id=marker.name.removesuffix('.submitted.json')
 manifest=json.loads((spool/(sample_id+'.json')).read_text());window=manifest['window']
 assert window['end_ns']-window['start_ns']==15_000_000_000
 assert manifest['training_eligible'] is False and manifest['label_status']=='unlabeled'
 reader=SequentialReader();reader.open(StorageOptions(uri=str(spool/(sample_id+'.mcap')),storage_id='mcap'),ConverterOptions('cdr','cdr'))
 counts={};last={};gaps={};source_stamps=0
 while reader.has_next():
  topic,data,stamp=reader.read_next();assert window['start_ns']<=stamp<=window['end_ns']
  counts[topic]=counts.get(topic,0)+1
  if topic in last:gaps[topic]=max(gaps.get(topic,0),stamp-last[topic])
  last[topic]=stamp
  if topic=='/joint_states' or topic in camera_topics:
   msg=deserialize_message(data,JointState if topic=='/joint_states' else CompressedImage)
   source=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
   assert source>0 and 0<=stamp-source<1_000_000_000;source_stamps+=1
 for topic in camera_topics+['/joint_states']:
  assert counts.get(topic,0)>2 and gaps[topic]<1_000_000_000
 assert counts.get('/arm_controller/commands',0)>0
 export_audit.append({'duration_s':15,'message_counts':counts,'preserved_source_stamps':source_stamps})
report['export_audit']=export_audit
(root/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
raise SystemExit(0 if report['passed'] else 1)
