#!/usr/bin/env python3
"""Build a trusted-fixture replay harness for MATLAB Coder's numeric frame ABI.

Generated C, MATLAB serializers and binaries go only to the selected build
folder. Input binaries are local benchmark fixtures, not an external API.
"""
import argparse
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('build', type=Path)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--matlab-root', type=Path, default=Path('/opt/MATLAB/R2026a'))
    args = parser.parse_args()
    build = args.build.resolve()
    header = (build / 'standaloneControllerFrame_types.h').read_text()
    types = {}
    for body, name in re.findall(r'typedef struct\s*\{([^}]+)\}\s*(\w+)\s*;', header):
        types[name] = body
    for name, body in re.findall(r'(?<!typedef )struct (emxArray_\w+)\s*\{([^}]+)\}\s*;', header):
        types[name] = body
    fields = {name: re.findall(r'^\s*(\w+)\s*(\*)?\s*(\w+)\s*;', body, re.M)
              for name, body in types.items()}
    emitted = set()
    c = ['#include "standaloneControllerFrame.h"', '#include "standaloneControllerFrame_emxAPI.h"',
         '#include "standaloneControllerFrame_initialize.h"', '#include "standaloneControllerFrame_terminate.h"',
         '#include "nativeControllerBenchmarkBridge.h"', '#include <stdio.h>', '#include <stdint.h>',
         '#include <string.h>', '#include <limits.h>', '#include <math.h>',
         'static void take(FILE *f, void *p, size_t z, size_t n) { if (fread(p,z,n,f)!=n) { fputs("Invalid fixture\\n",stderr); exit(2); } }']
    matlab = ['function writeStandaloneFixture(file,program)',
              "    fid=fopen(file,'w','ieee-le');assert(fid>=0);closeFile=onCleanup(@()fclose(fid));",
              '    w_struct0_T(fid,program);', 'end']
    primitive = {'double': ('double', 'double'), 'boolean_T': ('uint8', 'boolean_T'),
                 'int': ('int32', 'int'), 'creal_T': ('double', 'creal_T')}

    def emit(name):
        if name in emitted:
            return
        emitted.add(name)
        fs = fields[name]
        if name.startswith('emxArray_'):
            element = next(t for t, p, f in fs if f == 'data')
            if element not in primitive:
                emit(element)
            c.extend([f'static {name} *read_{name}(FILE *f) {{',
                      f'  {name} *a=calloc(1,sizeof(*a)); if(!a) exit(2);',
                      '  take(f,&a->numDimensions,4,1); if(a->numDimensions<1 || a->numDimensions>4) exit(2);',
                      '  a->size=calloc(a->numDimensions,sizeof(int)); if(!a->size) exit(2); take(f,a->size,4,a->numDimensions);',
                      '  size_t n=1; for(int i=0;i<a->numDimensions;++i) { if(a->size[i]<0 || a->size[i]>100000000 || n>100000000) exit(2); n*=a->size[i]; }',
                      '  if(n>100000000) exit(2); a->allocatedSize=(int)n; a->canFreeData=true;',
                      f'  a->data=calloc(n ? n : 1,sizeof({element})); if(!a->data) exit(2);'])
            if element in primitive:
                c.append(f'  take(f,a->data,sizeof({element}),n);')
            else:
                c.append(f'  for(size_t i=0;i<n;++i) read_{element}(f,&a->data[i]);')
            c.extend(['  return a;', '}'])
            matlab.extend([f'function w_{name}(fid,v,vector)',
                           '    if nargin<3,vector=false;end',
                           '    shape=size(v);if vector,shape=numel(v);end',
                           "    fwrite(fid,numel(shape),'int32');fwrite(fid,shape,'int32');"])
            if element == 'creal_T':
                matlab.append("    values=[real(v(:)).';imag(v(:)).'];fwrite(fid,values,'double');")
            elif element in primitive:
                matlab.append(f"    fwrite(fid,v(:),'{primitive[element][0]}');")
            else:
                matlab.append(f'    for i=1:numel(v),w_{element}(fid,v(i));end')
            matlab.append('end')
            return
        for element, pointer, field in fs:
            if element not in primitive:
                emit(element)
        c.append(f'static void read_{name}(FILE *f, {name} *v) {{')
        matlab.append(f'function w_{name}(fid,v)')
        sparse = {'d', 'rowidx', 'colidx'} <= {f for _, _, f in fs}
        if sparse:
            matlab.extend(["    [r,col,data]=find(v);[rows,columns]=size(v);",
                       "    v=struct('d',data,'rowidx',int32(r),'colidx',int32([1;1+cumsum(accumarray(col,1,[columns,1]))]), ...",
                       "        'm',int32(rows),'n',int32(columns),'maxnz',int32(numel(data)));" ])
        for element, pointer, field in fs:
            if element in primitive:
                c.append(f'  take(f,&v->{field},sizeof(v->{field}),1);')
                matlab.append(f"    fwrite(fid,v.{field},'{primitive[element][0]}');")
            elif pointer:
                c.append(f'  v->{field}=read_{element}(f);')
                force = ',true' if sparse and field in ('d', 'colidx', 'rowidx') else ''
                matlab.append(f'    w_{element}(fid,v.{field}{force});')
            else:
                c.append(f'  read_{element}(f,&v->{field});')
                matlab.append(f'    w_{element}(fid,v.{field});')
        c.append('}')
        matlab.append('end')
    emit('struct0_T')
    c.append(r'''
int main(int argc,char **argv) {
    if(argc<4) { fputs("Usage: controller-replay repeats warmups fixture...\n",stderr); return 2; }
    int repeats=atoi(argv[1]),warmups=atoi(argv[2]);
    if(repeats<1 || repeats>10000 || warmups<0 || warmups>10000) return 2;
    standaloneControllerFrame_initialize();
    for(int file=3;file<argc;++file) {
        for(int iteration=-warmups;iteration<repeats;++iteration) {
            FILE *f=fopen(argv[file],"rb");if(!f) return 2;
            struct0_T program={0};read_struct0_T(f,&program);fclose(f);
            emxArray_real_T *decision=emxCreate_real_T(0,1),*angles=emxCreate_real_T(0,1);
            double status=0,metrics[4]={0};
            double start=benchmark_clock();
            standaloneControllerFrame(&program,decision,angles,&status,metrics);
            double seconds=benchmark_clock()-start;
            if(iteration>=0) {
                printf("{\"file\":\"%s\",\"iteration\":%d,\"seconds\":%.17g,\"status\":%.0f,\"metrics\":[%.17g,%.17g,%.17g,%.17g]",
                    argv[file],iteration,seconds,status,metrics[0],metrics[1],metrics[2],metrics[3]);
                if(iteration==0) {
                    printf(",\"decision\":[");for(int k=0;k<decision->size[0];++k) printf("%s%.17g",k?",":"",decision->data[k]);
                    printf("],\"angles\":[");for(int k=0;k<angles->size[0];++k) printf("%s%.17g",k?",":"",angles->data[k]);
                    printf("]");
                }
                printf("}\n");fflush(stdout);
            }
            emxDestroy_struct0_T(program);emxDestroyArray_real_T(decision);emxDestroyArray_real_T(angles);
        }
    }
    standaloneControllerFrame_terminate();return 0;
}
''')
    (build / 'replay.c').write_text('\n'.join(c))
    (build / 'writeStandaloneFixture.m').write_text('\n'.join(matlab) + '\n')
    solver = args.root / 'solver/clarabel/rust_wrapper/target/release/libclarabel_c.a'
    subprocess.run(['gcc', '-O3', '-DNDEBUG', '-std=c11', '-D_POSIX_C_SOURCE=200809L',
                    '-I'+str(build), '-I'+str(args.matlab_root/'extern/include'), '-I'+str(args.root/'scripts'), str(build/'replay.c'),
                    str(build/'standaloneControllerFrame.a'), str(solver), '-ldl', '-lpthread', '-lm',
                    '-o', str(build/'controller-replay')], check=True)


if __name__ == '__main__':
    main()
