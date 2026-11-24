"""Local-only mock of grant/PUT/completion; all bytes and credentials synthetic."""
import http.server
import json
import os
from pathlib import Path
import time
import subprocess
import sys
import tempfile
import threading

requests=[]
connections=set()
objects={}
failed_completion=False


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version='HTTP/1.1'

    def log_message(self,*args):
        pass

    def reply(self,status,value):
        body=json.dumps(value).encode()
        self.send_response(status)
        self.send_header('Content-Type','application/json')
        self.send_header('Content-Length',str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        global failed_completion
        payload=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        sample=payload['manifest']['sample_id']
        requests.append(('POST',self.path,sample))
        connections.add(self.client_address)
        assert self.headers['X-Rearguard-Device-Secret']=='synthetic-secret'
        if self.path.endswith('/upload'):
            if sample=='sample-blocked':
                state=Path(payload['manifest']['test_state_path'])
                marker=Path(payload['manifest']['test_marker_path'])
                assert state.parent.parent.name.startswith('recording-upload-test-')
                assert marker.parent.parent.parent==state.parent.parent
                window={'id':'local','epoch':0,'status':'ready','start_ns':1_000_000_000,
                        'end_ns':16_000_000_000,'execution_sample':{'id':'sample-local-blocked'}}
                incident={k:v for k,v in window.items() if k!='execution_sample'}
                incident['id']='operator'
                temporary=state.with_suffix('.mock.tmp')
                temporary.write_text(json.dumps({'status':'observing','heartbeat_wall_ns':time.time_ns(),
                                                 'windows':[window,incident]}))
                temporary.replace(state)
                deadline=time.monotonic()+5
                while not marker.exists() and time.monotonic()<deadline:
                    time.sleep(.05)
                assert marker.exists(),'local preservation worker waited for HTTP completion'
            self.reply(200,{'upload_url':f'http://127.0.0.1:{self.server.server_port}/objects/{sample}',
                            'required_headers':{'If-None-Match':'*'}})
        elif sample=='sample-retry' and not failed_completion:
            failed_completion=True
            self.reply(503,{'error':'synthetic lost completion acknowledgement'})
        else:
            self.reply(200 if sample in objects else 404,{})

    def do_PUT(self):
        sample=self.path.rsplit('/',1)[-1]
        objects[sample]=self.rfile.read(int(self.headers['Content-Length']))
        requests.append(('PUT',self.path,sample))
        connections.add(self.client_address)
        assert self.headers['If-None-Match']=='*'
        self.reply(200,{})


server=http.server.ThreadingHTTPServer(('127.0.0.1',0),Handler)
thread=threading.Thread(target=server.serve_forever,daemon=True)
thread.start()
try:
    with tempfile.TemporaryDirectory(prefix='rearguard-upload-http-') as directory:
        result=subprocess.run([sys.argv[1],f'http://127.0.0.1:{server.server_port}'],
            env={**os.environ,'REARGUARD_STATE_DIR':directory,'NO_PROXY':'127.0.0.1','no_proxy':'127.0.0.1'},
            capture_output=True,text=True,timeout=60)
    print(result.stdout,result.stderr)
    assert result.returncode==0
    assert [x[0] for x in requests]==['POST','PUT','POST','POST','PUT','POST','POST','POST','PUT','POST'],requests
    assert requests[0][1].endswith('/upload'),requests
    assert requests[-1][1].endswith('/complete'),requests
    assert len(connections)==2,connections  # One persistent API connection, one object connection.
    print('HTTP checks passed: first attempt, completion recovery, durable backoff, completed skip, connection reuse, local work during blocked HTTP')
finally:
    server.shutdown()
    server.server_close()
