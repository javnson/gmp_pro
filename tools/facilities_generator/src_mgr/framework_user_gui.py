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
        self.root.title("GMP 工程框架配置器 (Project Framework Configurator)")
        self.root.geometry("1100x750")

        # 1. 解析核心路径
        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION！")
            self.root.destroy()
            return
            
        # 读取我们刚刚配置好的全局字典
        self.global_dic_path = Path(self.gmp_location) / "tools" / "facilities_generator" / "src_mgr" / "gmp_framework_dic.json"
        
        # 用户工程的本地配置文件 (保存在运行此脚本的当前目录)
        self.local_config_path = Path(os.getcwd()) / "gmp_framework_config.json"
        
        self.registry = {"roots": [], "modules": {}, "macros": {}}
        self.selected_modules = set() # 存储用户选中的独立模块，格式 "root_name|mod_key"
        
        self.load_global_dic()
        self.load_local_config()
        
        self.build_ui()
        self.refresh_tree()
        self.update_dashboard()

    # ================= 数据加载 =================
    def load_global_dic(self):
        if not self.global_dic_path.exists():
            messagebox.showerror("错误", f"找不到全局框架字典：\n{self.global_dic_path}\n请先使用开发者工具生成！")
            self.root.destroy()
            return
        try:
            with open(self.global_dic_path, 'r', encoding='utf-8') as f:
                self.registry = json.load(f)
        except Exception as e:
            messagebox.showerror("读取错误", f"全局字典解析失败: {e}")

    def load_local_config(self):
        """加载当前工程的历史配置"""
        if self.local_config_path.exists():
            try:
                with open(self.local_config_path, 'r', encoding='utf-8') as f:
                    local_config = json.load(f)
                    for item in local_config.get("selected_modules", []):
                        # 校验该模块是否在当前库中依然存在
                        root_name = item.get("root")
                        mod_key = item.get("module")
                        if root_name in self.registry.get("modules", {}) and mod_key in self.registry["modules"][root_name]:
                            self.selected_modules.add(f"{root_name}|{mod_key}")
            except Exception as e:
                print(f"警告: 本地配置文件读取失败 ({e})，将作为全新工程处理。")

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

        ttk.Label(left_frame, text="可用框架模块 (双击勾选/取消):", font=("Helvetica", 10, "bold")).pack(anchor=tk.W, pady=(0, 5))
        
        tree_scroll = ttk.Scrollbar(left_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree = ttk.Treeview(left_frame, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)
        
        self.tree.heading("#0", text=" 框架库列表", anchor=tk.W)
        self.tree.bind("<Double-1>", self.on_tree_double_click)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        # === 右侧：仪表盘面板 (Dashboard) ===
        right_frame = ttk.Frame(self.paned_window, padding=5)
        self.paned_window.add(right_frame, weight=2)

        # 顶部详情说明
        self.info_frame = ttk.LabelFrame(right_frame, text="节点详情说明", padding=10)
        self.info_frame.pack(fill=tk.X, pady=(0, 10))
        self.lbl_node_name = ttk.Label(self.info_frame, font=("Helvetica", 11, "bold"), text="请在左侧选择节点")
        self.lbl_node_name.pack(anchor=tk.W)
        self.lbl_node_desc = ttk.Label(self.info_frame, text="-", foreground="gray", wraplength=500)
        self.lbl_node_desc.pack(anchor=tk.W, pady=(5, 0))

        # 下方选项卡汇总区
        self.notebook = ttk.Notebook(right_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        def create_list_tab(title, color):
            frame = ttk.Frame(self.notebook, padding=5)
            self.notebook.add(frame, text=title)
            scroll = ttk.Scrollbar(frame)
            scroll.pack(side=tk.RIGHT, fill=tk.Y)
            lb = tk.Listbox(frame, yscrollcommand=scroll.set, fg=color, font=("Consolas", 9), bg="#fcfcfc")
            lb.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            scroll.config(command=lb.yview)
            return lb

        self.list_src = create_list_tab("📄 待同步源文件 (.c/.cpp)", "blue")
        self.list_inc = create_list_tab("📁 待镜像头文件 (.h/.hpp)", "green")
        
        # Include Dirs 专属面板带一键复制功能
        inc_dir_frame = ttk.Frame(self.notebook, padding=5)
        self.notebook.add(inc_dir_frame, text="⚙️ 编译器包含路径 (-I)")
        
        btn_copy = ttk.Button(inc_dir_frame, text="📋 一键复制全部路径 (供 IDE 使用)", command=self.copy_inc_dirs)
        btn_copy.pack(anchor=tk.E, pady=(0, 5))
        
        scroll_dirs = ttk.Scrollbar(inc_dir_frame)
        scroll_dirs.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_inc_dirs = tk.Listbox(inc_dir_frame, yscrollcommand=scroll_dirs.set, fg="#cc6600", font=("Consolas", 10, "bold"), bg="#fff9f0")
        self.list_inc_dirs.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_dirs.config(command=self.list_inc_dirs.yview)

        # === 底部：操作按钮 ===
        bottom_frame = ttk.Frame(self.root, padding=10)
        bottom_frame.pack(fill=tk.X)
        
        self.btn_sync = ttk.Button(bottom_frame, text="🚀 保存配置并生成工程代码", command=self.save_and_sync)
        self.btn_sync.pack(side=tk.RIGHT, ipadx=10, ipady=5, padx=5)
        
        ttk.Button(bottom_frame, text="💾 仅保存配置", command=self.save_local_config).pack(side=tk.RIGHT, ipadx=10, ipady=5)

    # ================= 智能依赖引擎 (核心逻辑) =================
    def get_all_dependencies(self, root_name, mod_key):
        """递归获取一个模块的完整依赖闭包"""
        closure = set()
        queue = [f"{root_name}|{mod_key}"]
        
        while queue:
            current = queue.pop(0)
            if current in closure: continue
            closure.add(current)
            
            c_root, c_mod = current.split('|', 1)
            deps = self.registry.get("modules", {}).get(c_root, {}).get(c_mod, {}).get("depends_on", [])
            
            for dep in deps:
                if dep not in closure:
                    queue.append(dep)
                    
        # 移除自己
        closure.discard(f"{root_name}|{mod_key}")
        return closure

    def check_can_uncheck(self, root_name, mod_key):
        """防呆检查：如果要取消勾选，是否有其他已选模块依赖了它？"""
        target = f"{root_name}|{mod_key}"
        conflicts = []
        
        for sel_mod in self.selected_modules:
            if sel_mod == target: continue
            s_root, s_mod = sel_mod.split('|', 1)
            deps = self.get_all_dependencies(s_root, s_mod)
            if target in deps:
                conflicts.append(sel_mod)
                
        return conflicts

    def set_module_checked(self, root_name, mod_key, state, silent=False):
        """安全地勾选/取消勾选一个模块及其依赖"""
        target = f"{root_name}|{mod_key}"
        
        if state: # 勾选
            self.selected_modules.add(target)
            # 自动勾选所有底层依赖
            deps = self.get_all_dependencies(root_name, mod_key)
            for dep in deps:
                self.selected_modules.add(dep)
        else: # 取消勾选
            conflicts = self.check_can_uncheck(root_name, mod_key)
            if conflicts:
                if not silent:
                    conflict_str = "\n".join([f" - {c.replace('|', ' / ')}" for c in conflicts])
                    messagebox.showwarning("依赖冲突", f"无法取消选中 '{mod_key}'！\n\n以下已选模块强依赖于它：\n{conflict_str}\n\n请先取消勾选上述模块。")
                return False # 取消失败
            else:
                self.selected_modules.discard(target)
                
        return True

    # ================= 树形交互 =================
    def refresh_tree(self):
        for item in self.tree.get_children(): self.tree.delete(item)

        for root_name in self.registry.get("roots", []):
            root_id = f"ROOT|{root_name}"
            self.tree.insert("", tk.END, iid=root_id, text=f"📦 {root_name}", open=True)
            
            modules = self.registry.get("modules", {}).get(root_name, {})
            sorted_keys = sorted(modules.keys())
            
            for mod_key in sorted_keys:
                mod_data = modules[mod_key]
                if mod_data.get("type") == "module":
                    parts = mod_key.split('|')
                    parent_id = root_id
                    
                    for i in range(len(parts) - 1):
                        folder_name = parts[i]
                        current_folder_path = "|".join(parts[:i+1])
                        folder_id = f"FOLDER|{root_name}|{current_folder_path}"
                        if not self.tree.exists(folder_id):
                            self.tree.insert(parent_id, tk.END, iid=folder_id, text=f"{UNCHECKED}📁 {folder_name}", open=True)
                        parent_id = folder_id

                    leaf_name = parts[-1]
                    mod_id = f"MOD|{root_name}|{mod_key}"
                    if leaf_name == "_internal":
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text=f"{UNCHECKED}⚙️ _internal")
                    else:
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text=f"{UNCHECKED}📄 {leaf_name}")

        self.update_tree_checkboxes()

    def update_tree_checkboxes(self, node=""):
        """递归刷新整棵树的 UI 复选框状态"""
        children = self.tree.get_children(node)
        if not children:
            if node.startswith("MOD|"):
                _, root_name, mod_key = node.split('|', 2)
                is_checked = f"{root_name}|{mod_key}" in self.selected_modules
                current_text = self.tree.item(node, "text")
                base_text = current_text.replace(CHECKED, "").replace(UNCHECKED, "")
                self.tree.item(node, text=f"{CHECKED if is_checked else UNCHECKED}{base_text}")
                return is_checked
            return False

        all_checked = True
        any_checked = False

        for child in children:
            is_checked = self.update_tree_checkboxes(child)
            if is_checked: any_checked = True
            else: all_checked = False

        if node and node.startswith("FOLDER|"):
            current_text = self.tree.item(node, "text")
            base_text = current_text.replace(CHECKED, "").replace(UNCHECKED, "").replace(PARTIAL, "")
            if all_checked: self.tree.item(node, text=f"{CHECKED}{base_text}")
            elif any_checked: self.tree.item(node, text=f"{PARTIAL}{base_text}")
            else: self.tree.item(node, text=f"{UNCHECKED}{base_text}")

        return all_checked

    def on_tree_double_click(self, event):
        """处理双击勾选事件 (支持包/文件夹级别连带勾选)"""
        region = self.tree.identify("region", event.x, event.y)
        if region == "cell" or region == "tree":
            item_id = self.tree.focus()
            if not item_id or item_id.startswith("ROOT|"): return

            current_text = self.tree.item(item_id, "text")
            target_state = False if CHECKED in current_text or PARTIAL in current_text else True

            if item_id.startswith("MOD|"):
                _, root_name, mod_key = item_id.split('|', 2)
                self.set_module_checked(root_name, mod_key, target_state)
                
            elif item_id.startswith("FOLDER|"):
                _, root_name, folder_path = item_id.split('|', 2)
                # 找出文件夹下所有的模块
                modules_under_folder = []
                for mod_key in self.registry["modules"][root_name].keys():
                    if mod_key.startswith(folder_path + '|'):
                        modules_under_folder.append(mod_key)
                        
                if target_state: # 勾选整个文件夹
                    for mod_key in modules_under_folder:
                        self.set_module_checked(root_name, mod_key, True)
                else: # 取消勾选整个文件夹
                    failed_count = 0
                    for mod_key in modules_under_folder:
                        if not self.set_module_checked(root_name, mod_key, False, silent=True):
                            failed_count += 1
                    if failed_count > 0:
                        messagebox.showwarning("部分取消失败", f"该文件夹下有 {failed_count} 个模块被外部依赖，无法取消勾选！")

            self.update_tree_checkboxes()
            self.update_dashboard() # 立即刷新仪表盘

    def on_tree_select(self, event):
        """处理单击事件：仅在右上角显示节点详情"""
        selected_items = self.tree.selection()
        if not selected_items: return
        node_id = selected_items[0]
        
        if node_id.startswith("ROOT|"):
            self.lbl_node_name.config(text=node_id.split('|')[1])
            self.lbl_node_desc.config(text="系统级根目录")
        elif node_id.startswith("FOLDER|"):
            _, r, f = node_id.split('|', 2)
            self.lbl_node_name.config(text=f.replace('|', ' / '))
            self.lbl_node_desc.config(text="包文件夹 (双击可全选其内部所有配置)")
        elif node_id.startswith("MOD|"):
            _, r, m = node_id.split('|', 2)
            desc = self.registry["modules"][r][m].get("description", "无描述")
            self.lbl_node_name.config(text=m.replace('|', ' / '))
            self.lbl_node_desc.config(text=desc)

    # ================= 仪表盘全局汇总 =================
    def update_dashboard(self):
        self.list_src.delete(0, tk.END)
        self.list_inc.delete(0, tk.END)
        self.list_inc_dirs.delete(0, tk.END)

        if not self.selected_modules:
            self.list_src.insert(tk.END, "(当前未选择任何模块)")
            return

        all_src_patterns = []
        all_inc_patterns = []
        all_inc_dirs_patterns = []

        # 1. 提取所有选中模块的规则
        for sel in self.selected_modules:
            root_name, mod_key = sel.split('|', 1)
            mod_data = self.registry["modules"][root_name][mod_key]
            
            all_src_patterns.extend(mod_data.get("src_patterns", []))
            all_inc_patterns.extend(mod_data.get("inc_patterns", []))
            all_inc_dirs_patterns.extend(mod_data.get("inc_dirs", []))

        # 2. 通配符解析与宏替换引擎
        def resolve_and_glob(patterns, is_dir_mode=False):
            matched = set()
            for pat in patterns:
                resolved_pat = pat
                for mac, val in self.registry.get("macros", {}).items():
                    resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
                
                pat_obj = Path(resolved_pat)
                
                if is_dir_mode:
                    if pat_obj.exists() and pat_obj.is_dir():
                        matched.add(pat_obj.as_posix())
                else:
                    if pat_obj.is_absolute():
                        anchor = pat_obj.anchor
                        rest = str(pat_obj.relative_to(anchor))
                        for f in Path(anchor).glob(rest):
                            if f.is_file(): matched.add(f.as_posix())
            return sorted(list(matched))

        # 3. 执行解析并展示
        src_files = resolve_and_glob(all_src_patterns)
        for f in src_files: self.list_src.insert(tk.END, f)
        self.notebook.tab(0, text=f"📄 待同步源文件 ({len(src_files)})")

        inc_files = resolve_and_glob(all_inc_patterns)
        for f in inc_files: self.list_inc.insert(tk.END, f)
        self.notebook.tab(1, text=f"📁 待镜像头文件 ({len(inc_files)})")
            
        dirs_files = resolve_and_glob(all_inc_dirs_patterns, is_dir_mode=True)
        for d in dirs_files: self.list_inc_dirs.insert(tk.END, d)
        self.notebook.tab(2, text=f"⚙️ 编译器包含路径 (-I) ({len(dirs_files)})")

    def copy_inc_dirs(self):
        """将 Include Dirs 拷贝到剪贴板，方便用户直接粘贴到 Keil/IAR"""
        dirs = self.list_inc_dirs.get(0, tk.END)
        if not dirs:
            messagebox.showinfo("提示", "当前没有包含路径可复制！")
            return
            
        clipboard_text = "\n".join(dirs)
        self.root.clipboard_clear()
        self.root.clipboard_append(clipboard_text)
        messagebox.showinfo("复制成功", f"已成功将 {len(dirs)} 条路径复制到剪贴板！\n可直接粘贴至 IDE 的 Include Paths 中。")

    # ================= 保存与同步 =================
    def save_local_config(self):
        config_data = {"selected_modules": []}
        for sel in sorted(list(self.selected_modules)):
            root_name, mod_key = sel.split('|', 1)
            config_data["selected_modules"].append({
                "root": root_name,
                "module": mod_key
            })
            
        try:
            with open(self.local_config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
            messagebox.showinfo("保存成功", f"工程配置已成功保存至:\n{self.local_config_path}")
            return True
        except Exception as e:
            messagebox.showerror("保存失败", f"无法写入配置文件: {e}")
            return False

    def save_and_sync(self):
        if self.save_local_config():
            messagebox.showinfo("准备就绪", "配置已保存！\n(下一步我们将开发核心的 backend 同步引擎，真正将这些文件复制到您的工程中！)")
            self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = FrameworkUserGUI(root)
    root.mainloop()