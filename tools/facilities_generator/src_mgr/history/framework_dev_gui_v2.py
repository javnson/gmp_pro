import os
import json
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from pathlib import Path

class FrameworkDevGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 核心框架资产管理器 (Framework SDK Manager)")
        self.root.geometry("950x650")

        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION")
            self.root.destroy()
            return
            
        # 【修复1】使用动态相对路径：强行绑定到当前 python 脚本所在的文件夹
        # 这样无论你把文件夹叫 src_mgr 还是 framework_mgr，只要 json 和 py 在一起就能找到！
        self.json_path = Path(__file__).parent / "gmp_framework_dic.json"
        
        self.registry = {"roots": [], "modules": {}}
        self.current_selected_node = None 
        self.all_available_modules = [] 
        
        self.build_ui()
        self.load_data()

    def build_ui(self):
        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # ================= 左侧：根模块与子模块树 =================
        left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(left_frame, weight=1)

        tree_frame = ttk.Frame(left_frame)
        tree_frame.pack(fill=tk.BOTH, expand=True)
        
        tree_scroll = ttk.Scrollbar(tree_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)
        self.tree.heading("#0", text="框架目录树", anchor=tk.W)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        # 左侧底部操作按钮
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="➕ 新增子模块", command=self.add_module).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(btn_frame, text="❌ 删除选中", command=self.delete_node).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

        # ================= 右侧：高级编辑面板 =================
        right_frame = ttk.Frame(self.paned_window, padding=10)
        self.paned_window.add(right_frame, weight=2)

        ttk.Label(right_frame, text="模块标识:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.lbl_module_id = ttk.Label(right_frame, font=("Helvetica", 10, "bold"), text="-")
        self.lbl_module_id.grid(row=0, column=1, sticky=tk.W, pady=5)

        ttk.Label(right_frame, text="模块描述:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.entry_desc = ttk.Entry(right_frame, width=40)
        self.entry_desc.grid(row=1, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="源文件规则 (src_patterns):\n(支持通配符, 每行一条)").grid(row=2, column=0, sticky=tk.NW, pady=5)
        self.txt_src = tk.Text(right_frame, height=4, width=40)
        self.txt_src.grid(row=2, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="头文件规则 (inc_patterns):\n(支持通配符, 每行一条)").grid(row=3, column=0, sticky=tk.NW, pady=5)
        self.txt_inc = tk.Text(right_frame, height=4, width=40)
        self.txt_inc.grid(row=3, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="底层依赖项 (depends_on):\n(按住 Ctrl 可多选)").grid(row=4, column=0, sticky=tk.NW, pady=5)
        
        dep_frame = ttk.Frame(right_frame)
        dep_frame.grid(row=4, column=1, sticky=tk.EW, pady=5)
        
        dep_scroll = ttk.Scrollbar(dep_frame)
        dep_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.listbox_deps = tk.Listbox(dep_frame, selectmode=tk.MULTIPLE, yscrollcommand=dep_scroll.set, height=6, exportselection=False)
        self.listbox_deps.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        dep_scroll.config(command=self.listbox_deps.yview)

        right_frame.rowconfigure(5, weight=1) 
        self.btn_save = ttk.Button(right_frame, text="💾 保存并更新模块", command=self.save_current_module)
        self.btn_save.grid(row=6, column=1, sticky=tk.E, pady=10)

        right_frame.columnconfigure(1, weight=1)
        self.clear_right_panel() 

    def load_data(self):
        if not self.json_path.exists():
            print(f"警告：未找到 {self.json_path}，初始化空配置。")
            self.registry = {"roots": ["core", "ctl", "cctl", "csp", "vctl"], "modules": {}}
            self.save_json()
        else:
            try:
                with open(self.json_path, 'r', encoding='utf-8') as f:
                    self.registry = json.load(f)
            except Exception as e:
                messagebox.showerror("读取错误", f"读取 JSON 失败: {e}")
                return

        self.refresh_tree()

    def update_available_modules(self):
        self.all_available_modules = []
        for root_name in self.registry.get("roots", []):
            modules = self.registry.get("modules", {}).get(root_name, {})
            for mod_name in modules.keys():
                self.all_available_modules.append(f"{root_name}|{mod_name}")
        self.all_available_modules.sort()

    def refresh_tree(self):
        for item in self.tree.get_children():
            self.tree.delete(item)

        self.update_available_modules()

        for root_name in self.registry.get("roots", []):
            root_id = f"ROOT|{root_name}"
            self.tree.insert("", tk.END, iid=root_id, text=root_name, open=True)
            
            modules = self.registry.get("modules", {}).get(root_name, {})
            for mod_name, mod_data in modules.items():
                if mod_data.get("type") == "module":
                    node_id = f"MOD|{root_name}|{mod_name}"
                    self.tree.insert(root_id, tk.END, iid=node_id, text=mod_name)

    def on_tree_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items: return
        node_id = selected_items[0]
        
        if node_id.startswith("ROOT|"):
            self.clear_right_panel()
            return
            
        # 【修复2】强制最多切割 2 次，这样 component|interface 的后面的 | 就不会被切碎了
        _, root_name, mod_name = node_id.split('|', 2)
        
        self.current_selected_node = (root_name, mod_name)
        mod_data = self.registry["modules"][root_name][mod_name]

        self.set_right_panel_state(tk.NORMAL)
        self.lbl_module_id.config(text=f"{root_name} | {mod_name}")
        
        self.entry_desc.delete(0, tk.END)
        self.entry_desc.insert(0, mod_data.get("description", ""))

        self.txt_src.delete(1.0, tk.END)
        self.txt_src.insert(tk.END, "\n".join(mod_data.get("src_patterns", [])))

        self.txt_inc.delete(1.0, tk.END)
        self.txt_inc.insert(tk.END, "\n".join(mod_data.get("inc_patterns", [])))

        self.listbox_deps.delete(0, tk.END)
        depends_on = mod_data.get("depends_on", [])
        for idx, available_mod in enumerate(self.all_available_modules):
            if available_mod == f"{root_name}|{mod_name}":
                continue
            self.listbox_deps.insert(tk.END, available_mod)
            if available_mod in depends_on:
                self.listbox_deps.selection_set(tk.END)

    def clear_right_panel(self):
        self.current_selected_node = None
        self.set_right_panel_state(tk.NORMAL)
        self.lbl_module_id.config(text="-")
        self.entry_desc.delete(0, tk.END)
        self.txt_src.delete(1.0, tk.END)
        self.txt_inc.delete(1.0, tk.END)
        self.listbox_deps.delete(0, tk.END)
        self.set_right_panel_state(tk.DISABLED)

    def set_right_panel_state(self, state):
        self.entry_desc.config(state=state)
        self.txt_src.config(state=state)
        self.txt_inc.config(state=state)
        self.btn_save.config(state=state)

    def save_current_module(self):
        if not self.current_selected_node: return
        root_name, mod_name = self.current_selected_node

        desc = self.entry_desc.get().strip()
        src_patterns = [f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()]
        inc_patterns = [f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()]
        
        selected_indices = self.listbox_deps.curselection()
        depends_on = [self.listbox_deps.get(i) for i in selected_indices]

        target_mod = self.registry["modules"][root_name][mod_name]
        target_mod["description"] = desc
        target_mod["src_patterns"] = src_patterns
        target_mod["inc_patterns"] = inc_patterns
        target_mod["depends_on"] = depends_on

        self.save_json()
        messagebox.showinfo("成功", f"模块 '{mod_name}' 配置已保存！")

    def add_module(self):
        selected = self.tree.selection()
        if not selected:
            messagebox.showwarning("提示", "请先在左侧选择一个根模块 (如 core, ctl)！")
            return
            
        node_id = selected[0]
        # 【修复2同样适用这里】防止提取根名时出错
        if node_id.startswith("ROOT|"):
            root_name = node_id.split('|', 1)[1]
        elif node_id.startswith("MOD|"):
            root_name = node_id.split('|', 2)[1]
            
        mod_name = simpledialog.askstring("新增子模块", f"在根模块 '{root_name}' 下新增模块:\n(支持用 '|' 表示子级，例如 component|pid)")
        if mod_name and mod_name.strip():
            mod_name = mod_name.strip()
            
            if root_name not in self.registry["modules"]:
                self.registry["modules"][root_name] = {}
                
            if mod_name in self.registry["modules"][root_name]:
                messagebox.showwarning("提示", "该模块已存在！")
                return
            
            self.registry["modules"][root_name][mod_name] = {
                "type": "module",
                "description": "新模块",
                "src_patterns": ["src/*.c"],
                "inc_patterns": ["**/*.h"],
                "depends_on": []
            }
            self.save_json()
            self.refresh_tree()
            self.tree.selection_set(f"MOD|{root_name}|{mod_name}")

    def delete_node(self):
        selected = self.tree.selection()
        if not selected: return
        node_id = selected[0]
        
        if node_id.startswith("ROOT|"):
            messagebox.showwarning("拒绝", "根模块 (Roots) 是系统固定的，不允许在此界面直接删除。")
            return
            
        if node_id.startswith("MOD|"):
            # 【修复2】
            _, root_name, mod_name = node_id.split('|', 2)
            if messagebox.askyesno("确认", f"确定要删除模块 '{mod_name}' 的注册信息吗？"):
                del self.registry["modules"][root_name][mod_name]
                self.save_json()
                self.refresh_tree()
                self.clear_right_panel()

    def save_json(self):
        with open(self.json_path, 'w', encoding='utf-8') as f:
            json.dump(self.registry, f, indent=4, ensure_ascii=False)

if __name__ == "__main__":
    root = tk.Tk()
    app = FrameworkDevGUI(root)
    root.mainloop()