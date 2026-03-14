import os
import json
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from pathlib import Path

class DevResourceMgrGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 外设资源管理器 V2 (开发者模式)")
        self.root.geometry("850x600")

        # 1. 确定文件路径
        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION")
            self.root.destroy()
            return
            
        self.json_path = Path(self.gmp_location) / "tools" / "facilities_generator" / "peripheral_mgr" / "gmp_peripheral_dic.json"
        
        self.registry = {}
        self.current_selected_node = None # 记录当前选中的节点路径 (Category, Module)
        self.current_selected_iid = None  # 记录当前在 Treeview 中的 ID
        
        # 2. 构建 UI
        self.build_ui()
        
        # 3. 加载数据
        self.load_data()

    def build_ui(self):
        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # ================= 左侧：树形目录与节点管理 =================
        left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(left_frame, weight=1)

        # 树形视图
        tree_frame = ttk.Frame(left_frame)
        tree_frame.pack(fill=tk.BOTH, expand=True)
        
        tree_scroll = ttk.Scrollbar(tree_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)
        self.tree.heading("#0", text="外设库目录树", anchor=tk.W)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        # 左侧底部操作按钮
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(btn_frame, text="➕ 新建分类", command=self.add_category).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(btn_frame, text="➕ 新建模块", command=self.add_module).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(btn_frame, text="❌ 删除选中", command=self.delete_node).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        
        ttk.Button(left_frame, text="🔍 扫描未分配文件 (待开发)", command=self.scan_orphan_files).pack(fill=tk.X, pady=2)

        # ================= 右侧：详细信息编辑面板 =================
        right_frame = ttk.Frame(self.paned_window, padding=10)
        self.paned_window.add(right_frame, weight=2)

        ttk.Label(right_frame, text="模块名称:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.entry_module_name = ttk.Entry(right_frame, font=("Helvetica", 10, "bold"))
        self.entry_module_name.grid(row=0, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="包含的 .c 文件:\n(每行一个相对路径)").grid(row=1, column=0, sticky=tk.NW, pady=5)
        self.txt_c_files = tk.Text(right_frame, height=4, width=40)
        self.txt_c_files.grid(row=1, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="包含的 .h 文件:\n(每行一个相对路径)").grid(row=2, column=0, sticky=tk.NW, pady=5)
        self.txt_h_files = tk.Text(right_frame, height=4, width=40)
        self.txt_h_files.grid(row=2, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="分类标签 (Tags):").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.entry_tags = ttk.Entry(right_frame, width=40)
        self.entry_tags.grid(row=3, column=1, sticky=tk.EW, pady=5)
        ttk.Label(right_frame, text="(多个标签请用逗号 ',' 隔开)", foreground="gray").grid(row=4, column=1, sticky=tk.W)

        ttk.Label(right_frame, text="模块描述:").grid(row=5, column=0, sticky=tk.NW, pady=5)
        self.txt_desc = tk.Text(right_frame, height=4, width=40)
        self.txt_desc.grid(row=5, column=1, sticky=tk.EW, pady=5)

        right_frame.rowconfigure(6, weight=1) # 占位推到底部
        self.btn_save = ttk.Button(right_frame, text="💾 保存当前模块修改", command=self.save_current_module)
        self.btn_save.grid(row=7, column=1, sticky=tk.E, pady=10)

        right_frame.columnconfigure(1, weight=1)
        self.clear_right_panel() # 初始状态清空并禁用右侧

    def load_data(self):
        if not self.json_path.exists():
            # 如果不存在，初始化一个空的结构
            self.registry = {"tree_nodes": {}}
        else:
            try:
                with open(self.json_path, 'r', encoding='utf-8') as f:
                    self.registry = json.load(f)
            except Exception as e:
                messagebox.showerror("读取错误", f"读取 JSON 失败: {e}")
                self.registry = {"tree_nodes": {}}

        self.refresh_tree()

    def refresh_tree(self):
        for item in self.tree.get_children():
            self.tree.delete(item)

        tree_nodes = self.registry.get("tree_nodes", {})
        for category_name, modules in tree_nodes.items():
            cat_id = self.tree.insert("", tk.END, iid=f"CAT|{category_name}", text=category_name, open=True)
            for module_name, module_data in modules.items():
                if module_data.get("type") == "module":
                    node_id = f"MOD|{category_name}|{module_name}"
                    self.tree.insert(cat_id, tk.END, iid=node_id, text=module_name)

    def on_tree_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items:
            return
            
        node_id = selected_items[0]
        self.current_selected_iid = node_id
        
        if node_id.startswith("CAT|"):
            self.clear_right_panel() # 选中父分类时不显示详情
            return

        # 解析选中的模块
        _, category_name, module_name = node_id.split('|')
        self.current_selected_node = (category_name, module_name)
        module_data = self.registry["tree_nodes"][category_name][module_name]

        # 启用并填充右侧 UI
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
        if not self.current_selected_node:
            return

        old_cat, old_mod = self.current_selected_node
        new_mod = self.entry_module_name.get().strip()
        
        if not new_mod:
            messagebox.showwarning("警告", "模块名称不能为空！")
            return

        # 处理重命名逻辑
        if new_mod != old_mod:
            if new_mod in self.registry["tree_nodes"][old_cat]:
                messagebox.showerror("错误", f"当前分类下已存在名为 '{new_mod}' 的模块！")
                return
            # 字典中替换 Key
            module_data = self.registry["tree_nodes"][old_cat].pop(old_mod)
            self.registry["tree_nodes"][old_cat][new_mod] = module_data
            self.current_selected_node = (old_cat, new_mod)

        # 获取并清洗文本框中的文件路径
        c_files = [f.strip() for f in self.txt_c_files.get(1.0, tk.END).split('\n') if f.strip()]
        h_files = [f.strip() for f in self.txt_h_files.get(1.0, tk.END).split('\n') if f.strip()]
        tags = [t.strip() for t in self.entry_tags.get().split(',') if t.strip()]
        desc = self.txt_desc.get(1.0, tk.END).strip()

        # 更新字典数据
        target_mod = self.registry["tree_nodes"][old_cat][new_mod]
        target_mod["c_files"] = c_files
        target_mod["h_files"] = h_files
        target_mod["tags"] = tags
        target_mod["description"] = desc

        self.save_json()
        
        # 刷新树并重新选中
        self.refresh_tree()
        new_iid = f"MOD|{old_cat}|{new_mod}"
        self.tree.selection_set(new_iid)
        messagebox.showinfo("成功", "保存成功！")

    # ================= 节点管理功能 =================
    def add_category(self):
        cat_name = simpledialog.askstring("新建分类", "请输入新的分类名称 (如 motor, power):")
        if cat_name and cat_name.strip():
            cat_name = cat_name.strip()
            if cat_name in self.registry["tree_nodes"]:
                messagebox.showwarning("提示", "该分类已存在！")
                return
            self.registry["tree_nodes"][cat_name] = {}
            self.save_json()
            self.refresh_tree()

    def add_module(self):
        selected = self.tree.selection()
        if not selected:
            messagebox.showwarning("提示", "请先在左侧选择一个父分类！")
            return
            
        node_id = selected[0]
        if node_id.startswith("CAT|"):
            cat_name = node_id.split('|')[1]
        else:
            cat_name = node_id.split('|')[1] # 如果选中的是模块，就在它所属的分类下建

        mod_name = simpledialog.askstring("新建模块", f"在分类 '{cat_name}' 下新建模块名称:")
        if mod_name and mod_name.strip():
            mod_name = mod_name.strip()
            if mod_name in self.registry["tree_nodes"][cat_name]:
                messagebox.showwarning("提示", "该模块已存在！")
                return
            
            # 创建标准结构
            self.registry["tree_nodes"][cat_name][mod_name] = {
                "type": "module",
                "c_files": [],
                "h_files": [],
                "tags": [cat_name],
                "description": f"New module: {mod_name}"
            }
            self.save_json()
            self.refresh_tree()
            # 自动选中新建的模块
            self.tree.selection_set(f"MOD|{cat_name}|{mod_name}")

    def delete_node(self):
        selected = self.tree.selection()
        if not selected:
            return
        node_id = selected[0]
        
        if node_id.startswith("CAT|"):
            cat_name = node_id.split('|')[1]
            if messagebox.askyesno("危险操作", f"确定要删除分类 '{cat_name}' 及其包含的所有模块吗？"):
                del self.registry["tree_nodes"][cat_name]
                self.save_json()
                self.refresh_tree()
                self.clear_right_panel()
                
        elif node_id.startswith("MOD|"):
            _, cat_name, mod_name = node_id.split('|')
            if messagebox.askyesno("确认", f"确定要删除模块 '{mod_name}' 的注册信息吗？\n(这不会删除实际的物理文件)"):
                del self.registry["tree_nodes"][cat_name][mod_name]
                self.save_json()
                self.refresh_tree()
                self.clear_right_panel()

    def save_json(self):
        self.json_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.json_path, 'w', encoding='utf-8') as f:
            json.dump(self.registry, f, indent=4, ensure_ascii=False)

    def scan_orphan_files(self):
        messagebox.showinfo("功能预告", "这里将弹出一个新窗口，遍历 core/dev/ 找出尚未在 JSON 中注册的 .c/.h 文件，方便你一键拖拽或指派。我们下一步就开发它！")

if __name__ == "__main__":
    root = tk.Tk()
    app = DevResourceMgrGUI(root)
    root.mainloop()