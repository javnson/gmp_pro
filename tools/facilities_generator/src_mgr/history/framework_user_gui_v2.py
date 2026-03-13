import os
import json
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path

# Unicode 字符用于模拟复选框状态
CHECKED = "☑ "
UNCHECKED = "☐ "
PARTIAL = "[-] "

class FrameworkUserGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 工程框架配置器 V2 (智能套餐与排他模式)")
        self.root.geometry("1150x800")

        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION！")
            self.root.destroy()
            return
            
        self.global_dic_path = Path(self.gmp_location) / "tools" / "facilities_generator" / "src_mgr" / "gmp_framework_dic.json"
        self.local_config_path = Path(os.getcwd()) / "gmp_framework_config.json"
        
        self.registry = {"roots": [], "modules": {}, "macros": {}}
        self.selected_modules = set() 
        self.current_selected_node = None # 记录当前单击激活的节点，用于查阅帮助文档
        
        self.load_global_dic()
        self.load_local_config()
        
        self.build_ui()
        self.refresh_tree()
        self.update_dashboard()

    # ================= 核心工具：路径与宏解析 =================
    def resolve_paths(self, patterns, is_dir_mode=False, return_absolute=False):
        """核心解析引擎：支撑文件预览和帮助文档的物理路径查找"""
        matched = set()
        base_dir = Path(self.gmp_location)
        macros = self.registry.get("macros", {})
        macros["GMP_PRO_LOCATION"] = base_dir.as_posix()

        for pat in patterns:
            resolved_pat = pat
            for mac, val in macros.items():
                resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)

            pat_obj = Path(resolved_pat)

            if pat_obj.is_absolute():
                search_base = Path(pat_obj.anchor)
                search_pattern = str(pat_obj.relative_to(pat_obj.anchor))
            else:
                search_base = base_dir
                search_pattern = str(pat_obj)

            if is_dir_mode:
                target = search_base / search_pattern
                if target.exists() and target.is_dir():
                    matched.add(target.resolve().as_posix() if return_absolute else target.as_posix())
            else:
                for f in search_base.glob(search_pattern):
                    if f.is_file():
                        matched.add(f.resolve().as_posix() if return_absolute else f.as_posix())

        return sorted(list(matched))

    # ================= 数据加载 =================
    def load_global_dic(self):
        if not self.global_dic_path.exists():
            messagebox.showerror("错误", f"找不到全局框架字典：\n{self.global_dic_path}")
            self.root.destroy()
            return
        try:
            with open(self.global_dic_path, 'r', encoding='utf-8') as f:
                self.registry = json.load(f)
        except Exception as e:
            messagebox.showerror("读取错误", f"全局字典解析失败: {e}")

    def load_local_config(self):
        if self.local_config_path.exists():
            try:
                with open(self.local_config_path, 'r', encoding='utf-8') as f:
                    local_config = json.load(f)
                    for item in local_config.get("selected_modules", []):
                        root_name = item.get("root")
                        mod_key = item.get("module")
                        if root_name in self.registry.get("modules", {}) and mod_key in self.registry["modules"][root_name]:
                            self.selected_modules.add(f"{root_name}|{mod_key}")
            except Exception as e:
                print(f"警告: 本地配置文件读取失败 ({e})")

    # ================= UI 构建 =================
    def build_ui(self):
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill=tk.X)
        ttk.Label(top_frame, text="当前工程路径:").pack(side=tk.LEFT)
        ttk.Label(top_frame, text=str(os.getcwd()), foreground="blue", font=("Helvetica", 10, "bold")).pack(side=tk.LEFT, padx=5)

        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # === 左侧：智能复选框依赖树 ===
        left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(left_frame, weight=1)

        ttk.Label(left_frame, text="可用框架模块 (双击支持三态切换):", font=("Helvetica", 10, "bold")).pack(anchor=tk.W, pady=(0, 5))
        
        tree_scroll = ttk.Scrollbar(left_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree = ttk.Treeview(left_frame, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)
        
        self.tree.heading("#0", text=" 框架库列表", anchor=tk.W)
        self.tree.bind("<Double-1>", self.on_tree_double_click)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        # === 右侧：仪表盘面板 ===
        right_frame = ttk.Frame(self.paned_window, padding=5)
        self.paned_window.add(right_frame, weight=2)

        # 详情说明与帮助文档区
        self.info_frame = ttk.LabelFrame(right_frame, text="节点详情与资源", padding=10)
        self.info_frame.pack(fill=tk.X, pady=(0, 10))
        
        info_inner = ttk.Frame(self.info_frame)
        info_inner.pack(fill=tk.X)
        
        text_info_frame = ttk.Frame(info_inner)
        text_info_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.lbl_node_name = ttk.Label(text_info_frame, font=("Helvetica", 11, "bold"), text="请在左侧选择节点")
        self.lbl_node_name.pack(anchor=tk.W)
        self.lbl_node_desc = ttk.Label(text_info_frame, text="-", foreground="gray", wraplength=500)
        self.lbl_node_desc.pack(anchor=tk.W, pady=(5, 0))
        
        # 帮助文档专用按钮
        self.btn_open_help = ttk.Button(info_inner, text="🌐 阅读说明文档", command=self.open_help_docs, state=tk.DISABLED)
        self.btn_open_help.pack(side=tk.RIGHT, padx=10, ipadx=5, ipady=5)

        # 选项卡汇总区
        self.notebook = ttk.Notebook(right_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # ------ Tab 1: 全局汇总 ------
        self.tab_summary = ttk.Frame(self.notebook, padding=5)
        self.notebook.add(self.tab_summary, text="📊 配置汇总清单")
        pw_summary = ttk.PanedWindow(self.tab_summary, orient=tk.VERTICAL)
        pw_summary.pack(fill=tk.BOTH, expand=True)

        frame_mods = ttk.Frame(pw_summary)
        ttk.Label(frame_mods, text="📦 当前工程已启用的独立模块清单:", font=("Helvetica", 9, "bold")).pack(anchor=tk.W)
        scroll_m = ttk.Scrollbar(frame_mods)
        scroll_m.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_summary_mods = tk.Listbox(frame_mods, yscrollcommand=scroll_m.set, bg="#fcfcfc", font=("Consolas", 10))
        self.list_summary_mods.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_m.config(command=self.list_summary_mods.yview)
        pw_summary.add(frame_mods, weight=1)

        frame_dirs = ttk.Frame(pw_summary)
        header_dirs = ttk.Frame(frame_dirs)
        header_dirs.pack(fill=tk.X, pady=(5, 2))
        ttk.Label(header_dirs, text="⚙️ 需添加至编译器的包含路径 (-I):", font=("Helvetica", 9, "bold"), foreground="#cc6600").pack(side=tk.LEFT)
        ttk.Button(header_dirs, text="📋 一键复制", command=self.copy_inc_dirs).pack(side=tk.RIGHT)
        
        scroll_d = ttk.Scrollbar(frame_dirs)
        scroll_d.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_inc_dirs = tk.Listbox(frame_dirs, yscrollcommand=scroll_d.set, fg="#cc6600", font=("Consolas", 10, "bold"), bg="#fff9f0")
        self.list_inc_dirs.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_d.config(command=self.list_inc_dirs.yview)
        pw_summary.add(frame_dirs, weight=1)

        # ------ Tab 2: 物理文件 ------
        self.tab_files = ttk.Frame(self.notebook, padding=5)
        self.notebook.add(self.tab_files, text="🗂️ 待同步文件详情")
        pw_files = ttk.PanedWindow(self.tab_files, orient=tk.VERTICAL)
        pw_files.pack(fill=tk.BOTH, expand=True)

        frame_src = ttk.Frame(pw_files)
        ttk.Label(frame_src, text="📄 待拷贝源文件 (.c / .cpp):", foreground="blue", font=("Helvetica", 9, "bold")).pack(anchor=tk.W)
        scroll_s = ttk.Scrollbar(frame_src)
        scroll_s.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_src = tk.Listbox(frame_src, yscrollcommand=scroll_s.set, fg="blue", font=("Consolas", 9), bg="#fcfcfc")
        self.list_src.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_s.config(command=self.list_src.yview)
        pw_files.add(frame_src, weight=1)

        frame_inc = ttk.Frame(pw_files)
        ttk.Label(frame_inc, text="📁 待镜像头文件 (.h / .hpp):", foreground="green", font=("Helvetica", 9, "bold")).pack(anchor=tk.W)
        scroll_i = ttk.Scrollbar(frame_inc)
        scroll_i.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_inc = tk.Listbox(frame_inc, yscrollcommand=scroll_i.set, fg="green", font=("Consolas", 9), bg="#fcfcfc")
        self.list_inc.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_i.config(command=self.list_inc.yview)
        pw_files.add(frame_inc, weight=1)

        # === 底部：操作按钮 ===
        bottom_frame = ttk.Frame(self.root, padding=10)
        bottom_frame.pack(fill=tk.X)
        self.btn_sync = ttk.Button(bottom_frame, text="🚀 保存配置并生成工程代码", command=self.save_and_sync)
        self.btn_sync.pack(side=tk.RIGHT, ipadx=10, ipady=5, padx=5)
        ttk.Button(bottom_frame, text="💾 仅保存配置", command=self.save_local_config).pack(side=tk.RIGHT, ipadx=10, ipady=5)

    # ================= 智能依赖引擎 (核心防呆逻辑) =================
    def get_all_dependencies(self, root_name, mod_key):
        """递归获取依赖闭包"""
        closure = set()
        queue = [f"{root_name}|{mod_key}"]
        while queue:
            current = queue.pop(0)
            if current in closure: continue
            closure.add(current)
            c_root, c_mod = current.split('|', 1)
            deps = self.registry.get("modules", {}).get(c_root, {}).get(c_mod, {}).get("depends_on", [])
            for dep in deps:
                if dep not in closure: queue.append(dep)
        closure.discard(f"{root_name}|{mod_key}")
        return closure

    def check_can_uncheck(self, root_name, mod_key):
        target = f"{root_name}|{mod_key}"
        conflicts = []
        
        # 1. 检查是否有上层模块依赖它
        for sel_mod in self.selected_modules:
            if sel_mod == target: continue
            s_root, s_mod = sel_mod.split('|', 1)
            deps = self.get_all_dependencies(s_root, s_mod)
            if target in deps: conflicts.append(sel_mod)

        # 2. 特殊防呆：如果目标是 _internal，且该文件夹下还有其他模块被选中，则绝对不准取消！
        if target.endswith("|_internal"):
            folder_prefix = target[:-10] # 截掉 '|_internal'
            for sel_mod in self.selected_modules:
                if sel_mod != target and sel_mod.startswith(folder_prefix + "|"):
                    conflicts.append(f"{sel_mod} (同包必须依赖其 _internal)")
                    
        return conflicts

    def set_module_checked(self, root_name, mod_key, state, silent=False):
        target = f"{root_name}|{mod_key}"
        
        if state: # 勾选逻辑
            # 【特性 6：CSP 目录单选排他性】
            if root_name == "csp":
                chip_family = mod_key.split('|')[0]
                to_uncheck = []
                for sel in self.selected_modules:
                    s_root, s_mod = sel.split('|', 1)
                    if s_root == "csp" and s_mod.split('|')[0] != chip_family:
                        to_uncheck.append((s_root, s_mod))
                # 强行取消其他芯片包
                for r, m in to_uncheck:
                    self.set_module_checked(r, m, False, silent=True)

            self.selected_modules.add(target)

            # 自动勾选配置的依赖
            for dep in self.get_all_dependencies(root_name, mod_key):
                self.selected_modules.add(dep)
                
            # 【特性 2：强制包含包级 _internal】
            parts = mod_key.split('|')
            for i in range(1, len(parts)):
                parent_internal = f"{'|'.join(parts[:i])}|_internal"
                if parent_internal in self.registry["modules"][root_name]:
                    self.selected_modules.add(f"{root_name}|{parent_internal}")

        else: # 取消勾选逻辑
            conflicts = self.check_can_uncheck(root_name, mod_key)
            if conflicts:
                if not silent:
                    conflict_str = "\n".join([f" - {c.replace('|', ' / ')}" for c in set(conflicts)])
                    messagebox.showwarning("依赖冲突拦截", f"无法取消选中 '{mod_key}'！\n\n原因：\n{conflict_str}")
                return False 
            else:
                self.selected_modules.discard(target)
                
        return True

    # ================= 树形交互与三态逻辑 =================
    def refresh_tree(self):
        for item in self.tree.get_children(): self.tree.delete(item)

        for root_name in self.registry.get("roots", []):
            root_id = f"ROOT|{root_name}"
            self.tree.insert("", tk.END, iid=root_id, text=f"📦 {root_name}", open=True)
            
            modules = self.registry.get("modules", {}).get(root_name, {})
            for mod_key in sorted(modules.keys()):
                mod_data = modules[mod_key]
                if mod_data.get("type") == "module":
                    parts = mod_key.split('|')
                    parent_id = root_id
                    
                    for i in range(len(parts) - 1):
                        folder_name = parts[i]
                        current_folder_path = "|".join(parts[:i+1])
                        folder_id = f"FOLDER|{root_name}|{current_folder_path}"
                        if not self.tree.exists(folder_id):
                            # 【特性 3：默认折叠 dev 文件夹】
                            is_open = False if folder_name == "dev" else True
                            self.tree.insert(parent_id, tk.END, iid=folder_id, text=f"{UNCHECKED}📁 {folder_name}", open=is_open)
                        parent_id = folder_id

                    leaf_name = parts[-1]
                    mod_id = f"MOD|{root_name}|{mod_key}"
                    if leaf_name == "_internal":
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text=f"{UNCHECKED}⚙️ _internal")
                    else:
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text=f"{UNCHECKED}📄 {leaf_name}")

        self.update_tree_checkboxes()

    def update_tree_checkboxes(self, node=""):
        children = self.tree.get_children(node)
        if not children:
            if node.startswith("MOD|"):
                _, root_name, mod_key = node.split('|', 2)
                is_checked = f"{root_name}|{mod_key}" in self.selected_modules
                base_text = self.tree.item(node, "text").replace(CHECKED, "").replace(UNCHECKED, "")
                self.tree.item(node, text=f"{CHECKED if is_checked else UNCHECKED}{base_text}")
                return is_checked
            return False

        all_checked, any_checked = True, False
        for child in children:
            if self.update_tree_checkboxes(child): any_checked = True
            else: all_checked = False

        if node and node.startswith("FOLDER|"):
            base_text = self.tree.item(node, "text").replace(CHECKED, "").replace(UNCHECKED, "").replace(PARTIAL, "")
            if all_checked: self.tree.item(node, text=f"{CHECKED}{base_text}")
            elif any_checked: self.tree.item(node, text=f"{PARTIAL}{base_text}")
            else: self.tree.item(node, text=f"{UNCHECKED}{base_text}")

        return all_checked

    def on_tree_double_click(self, event):
        """【特性 4：双击三态循环控制 (全选 -> 仅内部 -> 全不选)】"""
        region = self.tree.identify("region", event.x, event.y)
        if region not in ("cell", "tree"): return
        item_id = self.tree.focus()
        if not item_id or item_id.startswith("ROOT|"): return

        if item_id.startswith("MOD|"):
            _, root_name, mod_key = item_id.split('|', 2)
            target_state = False if CHECKED in self.tree.item(item_id, "text") else True
            self.set_module_checked(root_name, mod_key, target_state)
            
        elif item_id.startswith("FOLDER|"):
            _, root_name, folder_path = item_id.split('|', 2)
            
            # 搜集该文件夹下的所有模块和 internal 特殊件
            modules_in_folder = [m for m in self.registry["modules"][root_name].keys() if m.startswith(folder_path + '|')]
            internal_key = f"{folder_path}|_internal"
            has_internal = internal_key in modules_in_folder
            
            # 统计当前选中状态
            selected_in_folder = [m for m in modules_in_folder if f"{root_name}|{m}" in self.selected_modules]
            S = len(selected_in_folder)
            N = len(modules_in_folder)

            # 状态机运转
            if S == N and N > 0:
                # 状态1 (全选) -> 状态2 (仅保留_internal)
                for m in modules_in_folder:
                    if m != internal_key:
                        self.set_module_checked(root_name, m, False, silent=True)
                if has_internal:
                    self.set_module_checked(root_name, internal_key, True, silent=True)
            elif has_internal and S == 1 and selected_in_folder[0] == internal_key:
                # 状态2 (仅内部) -> 状态3 (全不选)
                self.set_module_checked(root_name, internal_key, False, silent=True)
            else:
                # 状态3 或 混合状态 -> 状态1 (全选)
                for m in modules_in_folder:
                    self.set_module_checked(root_name, m, True, silent=True)

        self.update_tree_checkboxes()
        self.update_dashboard()

    def on_tree_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items: return
        node_id = selected_items[0]
        
        self.btn_open_help.config(state=tk.DISABLED) # 默认失能
        
        if node_id.startswith("ROOT|"):
            self.current_selected_node = None
            self.lbl_node_name.config(text=node_id.split('|')[1])
            self.lbl_node_desc.config(text="系统级根目录")
        elif node_id.startswith("FOLDER|"):
            self.current_selected_node = None
            _, r, f = node_id.split('|', 2)
            self.lbl_node_name.config(text=f.replace('|', ' / '))
            self.lbl_node_desc.config(text="包文件夹 (双击可进行: 全选 -> 仅保留基础包 -> 全不选 的三态切换)")
        elif node_id.startswith("MOD|"):
            _, r, m = node_id.split('|', 2)
            self.current_selected_node = (r, m)
            mod_data = self.registry["modules"][r][m]
            
            self.lbl_node_name.config(text=m.replace('|', ' / '))
            self.lbl_node_desc.config(text=mod_data.get("description", "无描述"))
            
            # 【特性 1：动态点亮帮助文档按钮】
            if mod_data.get("help_docs"):
                self.btn_open_help.config(state=tk.NORMAL)

    def open_help_docs(self):
        if not self.current_selected_node: return
        r, m = self.current_selected_node
        docs = self.registry["modules"][r][m].get("help_docs", [])
        
        abs_paths = self.resolve_paths(docs, return_absolute=True)
        if not abs_paths:
            messagebox.showwarning("警告", "未能找到说明文档！请确保字典中配置的路径正确。")
            return
            
        for p in abs_paths:
            try: os.startfile(p)
            except Exception as e: print(f"打开文件失败 {p}: {e}")

    # ================= 仪表盘与汇总 =================
    def update_dashboard(self):
        self.list_summary_mods.delete(0, tk.END)
        self.list_inc_dirs.delete(0, tk.END)
        self.list_src.delete(0, tk.END)
        self.list_inc.delete(0, tk.END)

        if not self.selected_modules: return

        # 汇总展示 Tab 1
        for sel in sorted(list(self.selected_modules)):
            self.list_summary_mods.insert(tk.END, f"  ✅ {sel.replace('|', ' / ')}")

        all_src_patterns, all_inc_patterns, all_inc_dirs_patterns = [], [], []

        for sel in self.selected_modules:
            r, m = sel.split('|', 1)
            mod_data = self.registry["modules"][r][m]
            all_src_patterns.extend(mod_data.get("src_patterns", []))
            all_inc_patterns.extend(mod_data.get("inc_patterns", []))
            all_inc_dirs_patterns.extend(mod_data.get("inc_dirs", []))

        # 计算并展示 Include Dirs (-I)
        dirs_files = self.resolve_paths(all_inc_dirs_patterns, is_dir_mode=True)
        for d in dirs_files: self.list_inc_dirs.insert(tk.END, d)

        # 计算并展示 Tab 2 (具体文件)
        src_files = self.resolve_paths(all_src_patterns)
        for f in src_files: self.list_src.insert(tk.END, f)
        
        inc_files = self.resolve_paths(all_inc_patterns)
        for f in inc_files: self.list_inc.insert(tk.END, f)
        
        # 动态更新标签数字
        self.notebook.tab(0, text=f"📊 配置汇总清单 (模块:{len(self.selected_modules)})")
        self.notebook.tab(1, text=f"🗂️ 待同步文件清单 (共 {len(src_files) + len(inc_files)} 个)")

    def copy_inc_dirs(self):
        dirs = self.list_inc_dirs.get(0, tk.END)
        if not dirs:
            messagebox.showinfo("提示", "当前没有包含路径可复制！")
            return
        self.root.clipboard_clear()
        self.root.clipboard_append("\n".join(dirs))
        messagebox.showinfo("复制成功", f"已复制 {len(dirs)} 条路径！\n请将它们粘贴至 Keil/IAR 的 Include Paths 设置中。")

    # ================= 保存与同步 =================
    def save_local_config(self):
        config_data = {"selected_modules": []}
        for sel in sorted(list(self.selected_modules)):
            root_name, mod_key = sel.split('|', 1)
            config_data["selected_modules"].append({"root": root_name, "module": mod_key})
            
        try:
            with open(self.local_config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
            messagebox.showinfo("保存成功", f"工程配置已成功保存！")
            return True
        except Exception as e:
            messagebox.showerror("保存失败", f"无法写入配置文件: {e}")
            return False

    def save_and_sync(self):
        if self.save_local_config():
            messagebox.showinfo("准备就绪", "配置已保存！\n稍后执行 .bat 即可一键拉取所有文件并搭建本地镜像。")
            self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = FrameworkUserGUI(root)
    root.mainloop()
