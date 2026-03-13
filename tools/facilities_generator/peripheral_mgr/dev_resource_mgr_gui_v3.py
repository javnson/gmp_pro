import os
import json
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from pathlib import Path

class DevResourceMgrGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 外设资源管理器 V3 (带筛选与孤儿扫描)")
        self.root.geometry("900x650")

        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION")
            self.root.destroy()
            return
            
        self.json_path = Path(self.gmp_location) / "tools" / "facilities_generator" / "peripheral_mgr" / "gmp_peripheral_dic.json"
        
        self.registry = {}
        self.current_selected_node = None
        
        self.build_ui()
        self.load_data()

    def build_ui(self):
        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # ================= 左侧：树形目录与过滤 =================
        left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(left_frame, weight=1)

        # --- 新增：TAG 筛选区 ---
        filter_frame = ttk.Frame(left_frame)
        filter_frame.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(filter_frame, text="🏷️ TAG 筛选:").pack(side=tk.LEFT)
        self.entry_filter = ttk.Entry(filter_frame)
        self.entry_filter.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.entry_filter.bind('<Return>', lambda e: self.refresh_tree(self.entry_filter.get()))
        
        ttk.Button(filter_frame, text="筛选", width=6, command=lambda: self.refresh_tree(self.entry_filter.get())).pack(side=tk.LEFT, padx=1)
        ttk.Button(filter_frame, text="清除", width=6, command=self.clear_filter).pack(side=tk.LEFT)

        # --- 树形视图 ---
        tree_frame = ttk.Frame(left_frame)
        tree_frame.pack(fill=tk.BOTH, expand=True)
        
        tree_scroll = ttk.Scrollbar(tree_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)
        self.tree.heading("#0", text="外设库目录树", anchor=tk.W)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        # --- 左侧底部操作按钮 ---
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(btn_frame, text="➕ 新建分类", command=self.add_category).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(btn_frame, text="➕ 新建模块", command=self.add_module).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(btn_frame, text="❌ 删除选中", command=self.delete_node).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        
        ttk.Button(left_frame, text="🔍 扫描未分配文件 (孤儿文件)", command=self.scan_orphan_files).pack(fill=tk.X, pady=2)

        # ================= 右侧：详细信息编辑面板 =================
        right_frame = ttk.Frame(self.paned_window, padding=10)
        self.paned_window.add(right_frame, weight=2)

        ttk.Label(right_frame, text="模块名称:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.entry_module_name = ttk.Entry(right_frame, font=("Helvetica", 10, "bold"))
        self.entry_module_name.grid(row=0, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="包含的 .c 文件:\n(每行一个相对路径)").grid(row=1, column=0, sticky=tk.NW, pady=5)
        self.txt_c_files = tk.Text(right_frame, height=5, width=40)
        self.txt_c_files.grid(row=1, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="包含的 .h 文件:\n(每行一个相对路径)").grid(row=2, column=0, sticky=tk.NW, pady=5)
        self.txt_h_files = tk.Text(right_frame, height=5, width=40)
        self.txt_h_files.grid(row=2, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="分类标签 (Tags):").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.entry_tags = ttk.Entry(right_frame, width=40)
        self.entry_tags.grid(row=3, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="模块描述:").grid(row=4, column=0, sticky=tk.NW, pady=5)
        self.txt_desc = tk.Text(right_frame, height=5, width=40)
        self.txt_desc.grid(row=4, column=1, sticky=tk.EW, pady=5)

        right_frame.rowconfigure(5, weight=1) 
        self.btn_save = ttk.Button(right_frame, text="💾 保存当前模块修改", command=self.save_current_module)
        self.btn_save.grid(row=6, column=1, sticky=tk.E, pady=10)

        right_frame.columnconfigure(1, weight=1)
        self.clear_right_panel() 

    # ================= 核心数据加载与刷新 =================
    def load_data(self):
        if not self.json_path.exists():
            self.registry = {"tree_nodes": {}}
        else:
            try:
                with open(self.json_path, 'r', encoding='utf-8') as f:
                    self.registry = json.load(f)
            except Exception as e:
                messagebox.showerror("读取错误", f"读取 JSON 失败: {e}")
                self.registry = {"tree_nodes": {}}

        self.refresh_tree()

    def clear_filter(self):
        self.entry_filter.delete(0, tk.END)
        self.refresh_tree()

    def refresh_tree(self, tag_filter=""):
        for item in self.tree.get_children():
            self.tree.delete(item)

        tag_filter = tag_filter.strip().lower()
        tree_nodes = self.registry.get("tree_nodes", {})
        
        for category_name, modules in tree_nodes.items():
            cat_inserted = False
            cat_id = f"CAT|{category_name}"
            
            for module_name, module_data in modules.items():
                if module_data.get("type") == "module":
                    tags = [t.lower() for t in module_data.get("tags", [])]
                    
                    # 过滤逻辑：如果没有输入过滤词，或者过滤词在 tags 中
                    if not tag_filter or tag_filter in tags:
                        if not cat_inserted:
                            # 只有当分类下有符合条件的模块时，才插入分类节点
                            self.tree.insert("", tk.END, iid=cat_id, text=category_name, open=True)
                            cat_inserted = True
                            
                        node_id = f"MOD|{category_name}|{module_name}"
                        self.tree.insert(cat_id, tk.END, iid=node_id, text=module_name)

    # ... [此处保留之前的 on_tree_select, clear_right_panel, set_right_panel_state, save_current_module, add_category, add_module, delete_node, save_json 函数，为了节约篇幅未完全重复，请将它们保留在类中] ...
    
    # 注意：确保将 V2 版本的这几个函数完整保留并贴在这一块
    # [on_tree_select]
    # [clear_right_panel]
    # [set_right_panel_state]
    # [save_current_module]
    # [add_category]
    # [add_module]
    # [delete_node]
    # [save_json]

    # 为了方便你直接运行，我还是把它们补全在下面：
    def on_tree_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items: return
        node_id = selected_items[0]
        if node_id.startswith("CAT|"):
            self.clear_right_panel()
            return
        _, category_name, module_name = node_id.split('|')
        self.current_selected_node = (category_name, module_name)
        module_data = self.registry["tree_nodes"][category_name][module_name]

        self.set_right_panel_state(tk.NORMAL)
        self.entry_module_name.delete(0, tk.END)
        self.entry_module_name.insert(0, module_name)
        self.txt_c_files.delete(1.0, tk.END)
        self.txt_c_files.insert(tk.END, "\n".join(module_data.get("c_files", [])))
        self.txt_h_files.delete(1.0, tk.END)
        self.txt_h_files.insert(tk.END, "\n".join(module_data.get("h_files", [])))
        self.entry_tags.delete(0, tk.END)
        self.entry_tags.insert(0, ", ".join(module_data.get("tags", [])))
        self.txt_desc.delete(1.0, tk.END)
        self.txt_desc.insert(tk.END, module_data.get("description", ""))

    def clear_right_panel(self):
        self.current_selected_node = None
        self.set_right_panel_state(tk.NORMAL)
        self.entry_module_name.delete(0, tk.END)
        self.txt_c_files.delete(1.0, tk.END)
        self.txt_h_files.delete(1.0, tk.END)
        self.entry_tags.delete(0, tk.END)
        self.txt_desc.delete(1.0, tk.END)
        self.set_right_panel_state(tk.DISABLED)

    def set_right_panel_state(self, state):
        self.entry_module_name.config(state=state)
        self.txt_c_files.config(state=state)
        self.txt_h_files.config(state=state)
        self.entry_tags.config(state=state)
        self.txt_desc.config(state=state)
        self.btn_save.config(state=state)

    def save_current_module(self):
        if not self.current_selected_node: return
        old_cat, old_mod = self.current_selected_node
        new_mod = self.entry_module_name.get().strip()
        if not new_mod:
            messagebox.showwarning("警告", "模块名称不能为空！")
            return
        if new_mod != old_mod:
            if new_mod in self.registry["tree_nodes"][old_cat]:
                messagebox.showerror("错误", f"当前分类下已存在名为 '{new_mod}' 的模块！")
                return
            module_data = self.registry["tree_nodes"][old_cat].pop(old_mod)
            self.registry["tree_nodes"][old_cat][new_mod] = module_data
            self.current_selected_node = (old_cat, new_mod)

        c_files = [f.strip() for f in self.txt_c_files.get(1.0, tk.END).split('\n') if f.strip()]
        h_files = [f.strip() for f in self.txt_h_files.get(1.0, tk.END).split('\n') if f.strip()]
        tags = [t.strip() for t in self.entry_tags.get().split(',') if t.strip()]
        desc = self.txt_desc.get(1.0, tk.END).strip()

        target_mod = self.registry["tree_nodes"][old_cat][new_mod]
        target_mod["c_files"] = c_files
        target_mod["h_files"] = h_files
        target_mod["tags"] = tags
        target_mod["description"] = desc

        self.save_json()
        self.refresh_tree(self.entry_filter.get()) # 保持过滤状态刷新
        self.tree.selection_set(f"MOD|{old_cat}|{new_mod}")
        messagebox.showinfo("成功", "保存成功！")

    def add_category(self):
        cat_name = simpledialog.askstring("新建分类", "请输入新的分类名称:")
        if cat_name and cat_name.strip():
            cat_name = cat_name.strip()
            if cat_name in self.registry["tree_nodes"]: return
            self.registry["tree_nodes"][cat_name] = {}
            self.save_json()
            self.refresh_tree()

    def add_module(self):
        selected = self.tree.selection()
        if not selected: return
        node_id = selected[0]
        cat_name = node_id.split('|')[1]
        mod_name = simpledialog.askstring("新建模块", f"在 '{cat_name}' 下新建模块:")
        if mod_name and mod_name.strip():
            mod_name = mod_name.strip()
            if mod_name in self.registry["tree_nodes"][cat_name]: return
            self.registry["tree_nodes"][cat_name][mod_name] = {
                "type": "module", "c_files": [], "h_files": [], "tags": [cat_name], "description": ""
            }
            self.save_json()
            self.refresh_tree()
            self.tree.selection_set(f"MOD|{cat_name}|{mod_name}")

    def delete_node(self):
        selected = self.tree.selection()
        if not selected: return
        node_id = selected[0]
        if node_id.startswith("CAT|"):
            cat_name = node_id.split('|')[1]
            if messagebox.askyesno("警告", f"删除分类 '{cat_name}' 及所有模块？"):
                del self.registry["tree_nodes"][cat_name]
                self.save_json()
                self.refresh_tree()
                self.clear_right_panel()
        elif node_id.startswith("MOD|"):
            _, cat_name, mod_name = node_id.split('|')
            if messagebox.askyesno("确认", f"删除模块 '{mod_name}' 的注册信息？"):
                del self.registry["tree_nodes"][cat_name][mod_name]
                self.save_json()
                self.refresh_tree()
                self.clear_right_panel()

    def save_json(self):
        self.json_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.json_path, 'w', encoding='utf-8') as f:
            json.dump(self.registry, f, indent=4, ensure_ascii=False)


    # ================= 孤儿文件扫描核心逻辑 =================
    def scan_orphan_files(self):
        dev_dir = Path(self.gmp_location) / "core" / "dev"
        ignore_dirs = ['.git', 'build', 'common_includes']
        ignore_files = ['readme.md', 'prompt.h']
        
        # 1. 获取物理硬盘上的所有有效 .c 和 .h 文件
        physical_files = set()
        for root_dir, dirs, files in os.walk(dev_dir):
            dirs[:] = [d for d in dirs if d not in ignore_dirs] # 修改 dirs 会影响 os.walk 的遍历
            for file in files:
                if file not in ignore_files and file.endswith(('.c', '.h')):
                    filepath = Path(root_dir) / file
                    # 统一转换为相对于环境变量的路径
                    rel_path = filepath.relative_to(Path(self.gmp_location)).as_posix()
                    physical_files.add(rel_path)

        # 2. 获取 JSON 字典中已经注册的所有文件
        registered_files = set()
        for cat, modules in self.registry.get("tree_nodes", {}).items():
            for mod, data in modules.items():
                if data.get("type") == "module":
                    registered_files.update(data.get("c_files", []))
                    registered_files.update(data.get("h_files", []))

        # 3. 计算差集得出孤儿文件
        orphans = physical_files - registered_files

        if not orphans:
            messagebox.showinfo("扫描结果", "✨ 太棒了！所有的外设文件都已经记录在案，没有遗漏的孤儿文件。")
            return

        # 4. 弹出处理窗口
        self.open_orphan_assignment_window(sorted(list(orphans)))

    def open_orphan_assignment_window(self, orphan_list):
        top = tk.Toplevel(self.root)
        top.title(f"发现 {len(orphan_list)} 个未分配文件")
        top.geometry("600x400")
        
        # --- 顶部提示 ---
        ttk.Label(top, text="以下文件存在于硬盘上，但尚未被分配给任何模块。\n请选中它们，并指定一个归属模块：").pack(pady=5)
        
        # --- 列表框 (支持多选) ---
        list_frame = ttk.Frame(top)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10)
        
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        listbox = tk.Listbox(list_frame, selectmode=tk.EXTENDED, yscrollcommand=scrollbar.set)
        for item in orphan_list:
            listbox.insert(tk.END, item)
        listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=listbox.yview)

        # --- 底部控制区 ---
        ctrl_frame = ttk.Frame(top)
        ctrl_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Label(ctrl_frame, text="目标分类:").grid(row=0, column=0, padx=5)
        combo_cat = ttk.Combobox(ctrl_frame, state="readonly")
        combo_cat.grid(row=0, column=1, padx=5)
        
        ttk.Label(ctrl_frame, text="目标模块:").grid(row=0, column=2, padx=5)
        combo_mod = ttk.Combobox(ctrl_frame, state="readonly")
        combo_mod.grid(row=0, column=3, padx=5)

        # 动态加载分类和模块的逻辑
        categories = list(self.registry.get("tree_nodes", {}).keys())
        combo_cat['values'] = categories
        
        def on_cat_change(event):
            selected_cat = combo_cat.get()
            modules = list(self.registry["tree_nodes"][selected_cat].keys())
            combo_mod['values'] = modules
            combo_mod.set('') # 清空模块选择
            
        combo_cat.bind("<<ComboboxSelected>>", on_cat_change)

        def assign_files():
            selected_indices = listbox.curselection()
            if not selected_indices:
                messagebox.showwarning("提示", "请先在上面的列表中选中至少一个文件！", parent=top)
                return
            
            target_cat = combo_cat.get()
            target_mod = combo_mod.get()
            
            if not target_cat or not target_mod:
                messagebox.showwarning("提示", "请选择目标分类和目标模块！", parent=top)
                return
                
            # 执行分配
            target_data = self.registry["tree_nodes"][target_cat][target_mod]
            
            for i in selected_indices:
                file_path = listbox.get(i)
                if file_path.endswith('.c'):
                    if file_path not in target_data["c_files"]:
                        target_data["c_files"].append(file_path)
                elif file_path.endswith('.h'):
                    if file_path not in target_data["h_files"]:
                        target_data["h_files"].append(file_path)
            
            self.save_json()
            messagebox.showinfo("成功", f"已成功将 {len(selected_indices)} 个文件分配至 {target_mod}！", parent=top)
            
            # 刷新主界面，并关闭弹窗
            self.refresh_tree()
            top.destroy()
            
            # 自动选中目标节点方便预览
            self.tree.selection_set(f"MOD|{target_cat}|{target_mod}")

        btn_assign = ttk.Button(ctrl_frame, text="📥 分配至选定模块", command=assign_files)
        btn_assign.grid(row=0, column=4, padx=10)


if __name__ == "__main__":
    root = tk.Tk()
    app = DevResourceMgrGUI(root)
    root.mainloop()