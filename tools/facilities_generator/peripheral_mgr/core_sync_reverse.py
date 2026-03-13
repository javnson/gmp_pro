import os
import difflib
import shutil
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path

def run_reverse_sync_gui(parent_root, global_registry, selected_modules):
    """
    反向同步核心逻辑：
    扫描本地 gmp_peripheral_src 中的文件，与 core/dev 中的源文件比对。
    如果发现本地被修改过，则弹出一个高级的 Diff 对比窗口。
    """
    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    cwd = Path(os.getcwd())
    src_dest_dir = cwd / "gmp_peripheral_src"
    
    if not src_dest_dir.exists():
        messagebox.showinfo("提示", "本地暂无生成的源文件目录，无需反向同步。", parent=parent_root)
        return

    # 1. 构建映射：本地文件名 -> 核心库真实绝对路径
    local_to_lib_map = {}
    for module_key in selected_modules:
        cat, mod = module_key.split('|')
        try:
            mod_data = global_registry["tree_nodes"][cat][mod]
            for c_rel_path in mod_data.get("c_files", []):
                filename = Path(c_rel_path).name
                lib_abs_path = Path(gmp_location) / c_rel_path
                local_to_lib_map[filename] = lib_abs_path
        except KeyError:
            continue

    # 2. 扫描并比对文件内容
    modified_files_info = {} # 存储 {本地文件名: (本地路径, 核心库路径, diff文本)}
    
    for local_file in src_dest_dir.iterdir():
        if local_file.is_file() and local_file.name in local_to_lib_map:
            lib_file = local_to_lib_map[local_file.name]
            
            if not lib_file.exists():
                continue # 库文件丢失则跳过
            
            # 粗略筛选：通过修改时间判断本地是否更新
            if local_file.stat().st_mtime <= lib_file.stat().st_mtime:
                continue

            # 精确筛选：通过读取内容计算 Diff
            try:
                with open(lib_file, 'r', encoding='utf-8') as f1, open(local_file, 'r', encoding='utf-8') as f2:
                    lib_lines = f1.readlines()
                    local_lines = f2.readlines()
            except UnicodeDecodeError:
                # 兼容部分非 utf-8 编码的文件 (如 gbk)
                with open(lib_file, 'r', encoding='gbk', errors='ignore') as f1, open(local_file, 'r', encoding='gbk', errors='ignore') as f2:
                    lib_lines = f1.readlines()
                    local_lines = f2.readlines()

            # 生成 Unified Diff
            diff = list(difflib.unified_diff(
                lib_lines, local_lines, 
                fromfile=f"核心库 (库内旧版): {lib_file.name}", 
                tofile=f"当前工程 (本地新版): {local_file.name}", 
                n=3 # 上下文保留 3 行
            ))

            if diff: # 如果确实存在差异
                modified_files_info[local_file.name] = {
                    'local_path': local_file,
                    'lib_path': lib_file,
                    'diff_lines': diff
                }

    if not modified_files_info:
        messagebox.showinfo("反向同步检查", "太棒了！本地工程文件与核心库完全一致，没有需要反向提交的修改。", parent=parent_root)
        return

    # 3. 弹出差异确认界面
    show_diff_window(parent_root, modified_files_info)


def show_diff_window(parent_root, modified_files_info):
    """弹出带有代码高亮差异比对的窗口"""
    top = tk.Toplevel(parent_root)
    top.title("反向同步比对 (Diff Viewer)")
    top.geometry("1000x700")
    # 保证该窗口在最前端
    top.transient(parent_root)
    top.grab_set()

    paned = ttk.PanedWindow(top, orient=tk.HORIZONTAL)
    paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    # --- 左侧：被修改的文件列表 ---
    left_frame = ttk.Frame(paned)
    paned.add(left_frame, weight=1)
    
    ttk.Label(left_frame, text="检测到以下文件在本地被修改：\n(勾选确定要覆盖回库的文件)", foreground="red").pack(anchor=tk.W, pady=5)
    
    file_listbox = tk.Listbox(left_frame, selectmode=tk.MULTIPLE, font=("Helvetica", 11))
    file_listbox.pack(fill=tk.BOTH, expand=True)
    
    file_names = list(modified_files_info.keys())
    for name in file_names:
        file_listbox.insert(tk.END, name)
        file_listbox.selection_set(tk.END) # 默认全选

    # --- 右侧：Diff 高亮显示区 ---
    right_frame = ttk.Frame(paned)
    paned.add(right_frame, weight=3)
    
    ttk.Label(right_frame, text="代码差异明细 (绿底为新增行，红字为删除行):").pack(anchor=tk.W, pady=5)
    
    text_diff = tk.Text(right_frame, bg="#1e1e1e", fg="#d4d4d4", font=("Consolas", 10))
    text_diff.pack(fill=tk.BOTH, expand=True)
    
    # 配置 Text 的标签颜色以实现高亮
    text_diff.tag_config("added", background="#234b23", foreground="#4ec9b0") # 绿色高亮
    text_diff.tag_config("removed", foreground="#f14c4c", overstrike=1)       # 红色删除线
    text_diff.tag_config("header", foreground="#569cd6", font=("Consolas", 10, "bold")) # 蓝色头

    def on_select_file(event):
        """当点击左侧列表时，刷新右侧的 Diff 内容"""
        selection = file_listbox.curselection()
        if not selection: return
        
        # 仅显示点击的最后一个文件的 Diff
        clicked_idx = selection[-1]
        filename = file_names[clicked_idx]
        diff_lines = modified_files_info[filename]['diff_lines']
        
        text_diff.config(state=tk.NORMAL)
        text_diff.delete(1.0, tk.END)
        
        for line in diff_lines:
            if line.startswith('+++') or line.startswith('---'):
                text_diff.insert(tk.END, line, "header")
            elif line.startswith('@@'):
                text_diff.insert(tk.END, line, "header")
            elif line.startswith('+'):
                text_diff.insert(tk.END, line, "added")
            elif line.startswith('-'):
                text_diff.insert(tk.END, line, "removed")
            else:
                text_diff.insert(tk.END, line)
                
        text_diff.config(state=tk.DISABLED)

    # 绑定左侧点击事件
    file_listbox.bind("<<ListboxSelect>>", on_select_file)
    # 默认触发第一次点击显示
    if file_names:
        file_listbox.selection_set(0)
        file_listbox.event_generate("<<ListboxSelect>>")

    # --- 底部控制栏 ---
    bottom_frame = ttk.Frame(top)
    bottom_frame.pack(fill=tk.X, padx=10, pady=10)
    
    def apply_reverse_sync():
        selections = file_listbox.curselection()
        if not selections:
            messagebox.showwarning("提示", "未勾选任何文件！", parent=top)
            return
            
        if not messagebox.askyesno("最后确认", "该操作将把本地工程中修改过的代码\n强制覆盖到 GMP_PRO_LOCATION 的核心库中！\n是否确认操作？", parent=top):
            return
            
        success_count = 0
        for idx in selections:
            filename = file_names[idx]
            local_path = modified_files_info[filename]['local_path']
            lib_path = modified_files_info[filename]['lib_path']
            try:
                # 执行反向覆盖复制
                shutil.copy2(local_path, lib_path)
                success_count += 1
            except Exception as e:
                messagebox.showerror("覆盖失败", f"文件 {filename} 覆盖失败: {e}", parent=top)
                
        messagebox.showinfo("同步成功", f"成功将 {success_count} 个修改提交回核心外设库！", parent=top)
        top.destroy()

    btn_apply = ttk.Button(bottom_frame, text="⚠️ 确认将勾选的文件覆盖回核心库", command=apply_reverse_sync)
    btn_apply.pack(side=tk.RIGHT, ipadx=10, ipady=5)
    
    btn_cancel = ttk.Button(bottom_frame, text="取消", command=top.destroy)
    btn_cancel.pack(side=tk.RIGHT, padx=10)