#!/usr/bin/env python3
"""Fetch pinned official CPU SDK for reproducible Linux x86_64 release builds."""
import argparse
import hashlib
from pathlib import Path
import tempfile
import urllib.request
import zipfile

VERSION='1.31.1'
URL='https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/1.31.1/microsoft.ml.onnxruntime.1.31.1.nupkg'
SHA256='6871ccbc97f3d5bdb4095a1dcec33aa78d56f6ed8de5980ef5fa1d9f2e7c679a'


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--destination',type=Path,required=True)
    destination=parser.parse_args().destination;destination.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='ort-sdk-') as root:
        archive=Path(root)/'sdk.nupkg';urllib.request.urlretrieve(URL,archive)
        with archive.open('rb') as stream:actual=hashlib.file_digest(stream,'sha256').hexdigest()
        if actual!=SHA256:raise ValueError('official native SDK checksum mismatch')
        with zipfile.ZipFile(archive) as source:
            for name in source.namelist():
                if name in ('LICENSE','ThirdPartyNotices.txt','Privacy.md'):directory='share/onnxruntime'
                elif name.startswith('build/native/include/') and name.endswith('.h'):directory='include'
                elif name in ('runtimes/linux-x64/native/libonnxruntime.so','runtimes/linux-x64/native/libonnxruntime_providers_shared.so'):directory='lib'
                else:continue
                target=destination/directory/Path(name).name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(source.read(name))
    for name in ('libonnxruntime.so.1','libonnxruntime.so.'+VERSION):
        target=destination/'lib'/name
        if not target.exists():target.symlink_to('libonnxruntime.so')
    print('Pinned ONNX Runtime '+VERSION+' native SDK: '+str(destination))


if __name__=='__main__':main()
