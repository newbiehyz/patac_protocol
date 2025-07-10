import json,os
import argparse
import datetime
import shutil
import filecmp

def sync_folders(hyper_protocol_folder, auto_protocol_folder):

    hyper_files_names = []
    auto_files_names= []



    for root, dirs, files in os.walk(hyper_protocol_folder):
        for file in files:
            file_path = os.path.join(root, file)
            hyper_files_names.append(file_path)
            auto_files_names.append(file_path)

    for i in range(len(hyper_files_names)):
        f1 = hyper_files_names[i]
        f2 = auto_files_names[i].replace('hyper','autodrive')

        exclude_files = []
        for i in content["exclusion_files"]:
            exclude_files.append(os.path.join(os.getcwd(),'..',i))
        # print(exclude_files)

        if os.path.exists(f2):
            if f2.endswith('txt') or f2.endswith('pb.h') or f2.endswith('pb.cc'):
                pass
            elif f1 in exclude_files:
                pass
            elif(filecmp.cmp(f1,f2) == False):
                os.remove(f1)
                shutil.copyfile(f2,f1)
                print('copy file from ' + f2 + '  to  ' + f1)

    

if __name__ == '__main__':


    conf_path = os.path.join(os.getcwd(),'conf/settings.json')
    with open(conf_path,"r",encoding="utf-8") as f:
        content = json.load(f)

    for i in range(len(content["auto_protocol_files"])):

        auto_protocol_folder = os.path.join(os.getcwd(),'..',content["auto_protocol_files"][i])
        hyper_protocol_folder = os.path.join(os.getcwd(),'..',content["hyper_protocol_files"][i])
        sync_folders(hyper_protocol_folder, auto_protocol_folder)
    
