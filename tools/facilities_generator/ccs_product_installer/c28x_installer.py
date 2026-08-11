import os
import shutil
import json
import sys

# =====================================================================
# 1. 变量定义阶段 (Variables Definition)
# =====================================================================
# 后缀名称，用于避免同名冲突
suffix_name = "C28x"

# 目标生成地址的相对路径
target_dir_suffix = "csp/c28x_syscfg"

# source code template 后缀
suffix = '.xdt'


# =====================================================================
# 2. 环境检查与基础路径获取
# =====================================================================
# Get GMP installation path
gmp_pro_location = os.getenv('GMP_PRO_LOCATION')

# Check if GMP path has registered.
if gmp_pro_location is not None:
    print(f"\033[92m[INFO]\033[00m Environment variable GMP_PRO_LOCATION is: {gmp_pro_location}")
else:
    print("\033[91m[ERROR]\033[00m Environment variable GMP_PRO_LOCATION unknown. Reinstall this software may solve this problem.")
    sys.exit(1)

# 生成目标完整路径 (替换为正斜杠防止 CCS 解析报错)
target_base_path = os.path.join(gmp_pro_location, target_dir_suffix)
gmp_pro_loc_unix = gmp_pro_location.replace('\\', '/')
target_base_path_unix = target_base_path.replace('\\', '/')


# =====================================================================
# 3. 加载本地 Product Info 配置文件
# =====================================================================
# 获取当前 python 脚本所在的文件夹路径
current_dir = os.path.dirname(os.path.abspath(__file__))
product_info_file = os.path.join(current_dir, 'GMP_CCS_Product_Info.json')

if not os.path.exists(product_info_file):
    print(f"\033[91m[ERROR]\033[00m Configuration file not found: {product_info_file}")
    sys.exit(1)

with open(product_info_file, 'r', encoding='utf-8') as f:
    product_info = json.load(f)

# 读取并附加后缀
library_version = product_info.get("library_version", "1.00.00.00")

# 格式化 library_name (例如: GMP Pro -> gmp_pro_c28x)
raw_lib_name = product_info.get("library_name", "gmp_pro").replace(" ", "_").lower()
library_name = f"{raw_lib_name}_{suffix_name.lower()}"

# 格式化 display_name (例如: General Motor Platform Pro - C28x)
display_name = f"{product_info.get('display_name', 'General Motor Platform')} - {suffix_name}"

description = product_info.get("description", "General Motor Platform framework.")


# =====================================================================
# 4. JSON 模板定义
# =====================================================================
# Container of facilities
facilities_json = os.path.join(gmp_pro_location, 'tools', 'facilities_generator', 'json', 'facilities.json')

# product config file
config_file_template = {
    "name": library_name,
    "displayName": display_name,
    "version": library_version,
    "documentationPath": gmp_pro_loc_unix + "/manual",
    "includePaths": [
        gmp_pro_loc_unix,
        target_base_path_unix
    ],
    "devices": [
        "F28004x", "F2837xD", "F2837xS", "F2807x", "F2838x",
        "F28002x", "F28003x", "F28P65x", "F28P55x", "GMP_NULL_DEVICE"
    ],
    "minToolVersion": "1.21.0"
}

# tirex config file
tirex_package_template = [{
    "rootMacroName": f"GMP_PRO_{suffix_name.upper()}_SDK_INSTALL_DIR",
    "compilerIncludePath": [
        gmp_pro_loc_unix,
        target_base_path_unix
    ],
    "compilerSymbols": [],
    "linkerSearchPath": []
}]

# ccs package config file
tirex_config_template = [{
    "id": f"GMP-PRO-SDK-{suffix_name.upper()}",
    "name": display_name,
    "version": library_version,
    "type": "software",
    "image": gmp_pro_loc_unix + "/manual/icon/GMP_LOGO.png",
    "license": gmp_pro_loc_unix + "/LICENSE.txt",
    "description": description,
    "tags": [ "GMP", suffix_name ],
    "dependencies": [
        {
            "packageId": "C2000WARE",
            "version": "5.03.00.00",
            "require": "mandatory"
        },
        {
            "packageId": "sysconfig",
            "version": "1.21.0",
            "require": "mandatory"
        }
    ],
    "metadataVersion": "3.1.0"
}]


######################################################################
# Here is the implement function for GMP facilities

def gen_facilities_config_files(fac_data): 
    # init record
    records = [] 

    # target script file (现在使用顶层定义的 target_base_path)
    config_path = os.path.join(target_base_path, '.metadata')
    
    # ensure config path is exists
    if not os.path.exists(config_path):
        os.makedirs(config_path)

    # For all folder objects
    if 'sourceDir' in fac_data:
        for folder_item in fac_data['sourceDir'] :
            # source folder
            src_dir = os.path.join(gmp_pro_location, folder_item, 'src')

            # target path (仍然在 GMP_PRO_LOCATION 目录下生成 .meta，或者你需要也可以改到 target_base_path)
            dest_dir = os.path.join(gmp_pro_location, folder_item, '.meta')

             # ensure target dir is exists
            if not os.path.exists(dest_dir):
                os.makedirs(dest_dir)
                
            # For all file objects
            # for root, dirs, files in os.walk(src_dir):
            #    for file in files:
            #        # construct full file path
            #        src_file_path = os.path.join(root, file)
            #        dest_file_path = os.path.join(dest_dir, f"{os.path.splitext(file)[0]}{suffix}")
            #
            #        # copy
            #        shutil.copy2(src_file_path, dest_file_path)
            #        
            #        # record
            #        record = {
            #            "name": folder_item + f"{os.path.splitext(file)[0]}{suffix}" ,
            #            "outputPath": file,
            #            "alwaysRun": True
            #        }
            #        records.append(record)
            #
            #        print('\033[93m[INFO]\033[00m ' + dest_file_path + ' is generated.')

    # generate config files
    config_file_template["templates"] = records

    # generate product.json
    with open(os.path.join(config_path, 'product.json'), 'w', encoding='utf-8') as f:
        json.dump(config_file_template, f, indent=4)

    # tirex config files
    tirex_file_path = os.path.join(config_path, '.tirex')

    # ensure config path is exists
    if not os.path.exists(tirex_file_path):
        os.makedirs(tirex_file_path)

    # config tirex resource 
    # 如果 facilities_json 里面定义了覆盖配置，可在这里覆盖（如果没有则保留上文定义好的默认值）
    if 'rootMacroName' in fac_data:
        tirex_package_template[0]['rootMacroName'] = fac_data['rootMacroName']

    # generate tirex config files
    with open(os.path.join(tirex_file_path, 'package.ccs.json'), 'w', encoding='utf-8') as f:
        json.dump(tirex_package_template, f, indent=4)

    with open(os.path.join(tirex_file_path, 'package.tirex.json'), 'w', encoding='utf-8') as f:
        json.dump(tirex_config_template, f, indent=4)


######################################################################
# Start here
with open(facilities_json, 'r', encoding='utf-8') as f:
    data = json.load(f)

# 因为已经拆分脚本，假设设施 JSON 中包含的依然是类似不同模块的数据
for name in data.get("facilities", {}):
    fac_item = data["facilities"][name] if isinstance(data["facilities"], dict) else name
    # Say something
    print(f"\r\n\r\n\033[93m[INFO]\033[00m facilities: {fac_item.get('name', 'Unknown')} in {target_dir_suffix} is generating...")

    # generate config file for target 
    gen_facilities_config_files(fac_item)