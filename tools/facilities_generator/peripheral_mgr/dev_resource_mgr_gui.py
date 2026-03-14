import os
import json
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path

class DevResourceMgrGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 外设资源管理器 (开发者模式)")
        self.root.geometry("800x500")

        # 1. 确定文件路径
        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION")
            self.root.destroy()
            return
            
        self.json_path = Path(self.gmp_location) / "tools" / "facilities_generator" / "peripheral_mgr" / "gmp_peripheral_dic.json"
        
        self.registry = {}
        self.current_selected_node = None # 记录当前选中的节点路径
        
        # 2. 构建 UI
        self.build_ui()
        
        # 3. 加载数据
        self.load_data()

    def build_ui(self):
        # 使用 PanedWindow 左右分栏
        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # --- 左侧：树形目录 ---
        left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(left_frame, weight=1)

        # 添加滚动条
        tree_scroll = ttk.Scrollbar(left_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree = ttk.Treeview(left_frame, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)

        self.tree.heading("#0", text="外设库目录树", anchor=tk.W)
        # 绑定树节点的点击事件
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        # --- 右侧：详细信息编辑面板 ---
        right_frame = ttk.Frame(self.paned_window, padding=10)
        self.paned_window.add(right_frame, weight=2)

        ttk.Label(right_frame, text="模块名称:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.lbl_module_name = ttk.Label(right_frame, text="-", font=("Helvetica", 10, "bold"))
        self.lbl_module_name.grid(row=0, column=1, sticky=tk.W, pady=5)

        ttk.Label(right_frame, text="包含的 .c 文件:").grid(row=1, column=0, sticky=tk.NW, pady=5)
        self.txt_c_files = tk.Text(right_frame, height=3, width=40, state=tk.DISABLED, bg="#f0f0f0")
        self.txt_c_files.grid(row=1, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="包含的 .h 文件:").grid(row=2, column=0, sticky=tk.NW, pady=5)
        self.txt_h_files = tk.Text(right_frame, height=3, width=40, state=tk.DISABLED, bg="#f0f0f0")
        self.txt_h_files.grid(row=2, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="分类标签 (Tags):").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.entry_tags = ttk.Entry(right_frame, width=40)
        self.entry_tags.grid(row=3, column=1, sticky=tk.EW, pady=5)
        ttk.Label(right_frame, text="(多个标签请用逗号 ',' 隔开)", foreground="gray").grid(row=4, column=1, sticky=tk.W)

        ttk.Label(right_frame, text="模块描述:").grid(row=5, column=0, sticky=tk.NW, pady=5)
        self.txt_desc = tk.Text(right_frame, height=5, width=40)
        self.txt_desc.grid(row=5, column=1, sticky=tk.EW, pady=5)

        # 占位行，用于把按钮推到底部
        right_frame.rowconfigure(6, weight=1)

        self.btn_save = ttk.Button(right_frame, text="保存当前模块修改", command=self.save_current_module)
        self.btn_save.grid(row=7, column=1, sticky=tk.E, pady=10)

        # 设置列的权重，让输入框可以随窗口拉伸
        right_frame.columnconfigure(1, weight=1)

    def load_data(self):
        if not self.json_path.exists():
            messagebox.showerror("错误", f"找不到字典文件: {self.json_path}")
            return

        try:
            with open(self.json_path, 'r', encoding='utf-8') as f:
                self.registry = json.load(f)
        except Exception as e:
            messagebox.showerror("读取错误", f"读取 JSON 失败: {e}")
            return

        # 清空树
        for item in self.tree.get_children():
            self.tree.delete(item)

        tree_nodes = self.registry.get("tree_nodes", {})
        
        # 遍历分类 (如 _Root_Files, sensor, dac)
        for category_name, modules in tree_nodes.items():
            cat_id = self.tree.insert("", tk.END, text=category_name, open=True)
            
            # 遍历模块 (如 bh1750, bmp280)
            for module_name, module_data in modules.items():
                if module_data.get("type") == "module":
                    # 使用 'category|module' 作为唯一标识符
                    node_id = f"{category_name}|{module_name}"
                    self.tree.insert(cat_id, tk.END, iid=node_id, text=module_name)

    def on_tree_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items:
            return
            
        node_id = selected_items[0]
        
        # 如果选中的是顶级目录（没有 '|' 符号），则不处理
        if '|' not in node_id:
            self.clear_right_panel()
            return

        category_name, module_name = node_id.split('|')
        self.current_selected_node = (category_name, module_name)
        
        # 获取该模块的数据
        module_data = self.registry["tree_nodes"][category_name][module_name]

        # 更新右侧 UI
        self.lbl_module_name.config(text=module_name)
        
        self.txt_c_files.config(state=tk.NORMAL)
        self.txt_c_files.delete(1.0, tk.END)
        self.txt_c_files.insert(tk.END, "\n".join(module_data.get("c_files", [])))
        self.txt_c_files.config(state=tk.DISABLED)

        self.txt_h_files.config(state=tk.NORMAL)
        self.txt_h_files.delete(1.0, tk.END)
        self.txt_h_files.insert(tk.END, "\n".join(module_data.get("h_files", [])))
        self.txt_h_files.config(state=tk.DISABLED)

        self.entry_tags.delete(0, tk.END)
        self.entry_tags.insert(0, ", ".join(module_data.get("tags", [])))

        self.txt_desc.delete(1.0, tk.END)
        self.txt_desc.insert(tk.END, module_data.get("description", ""))

    def clear_right_panel(self):
        self.current_selected_node = None
        self.lbl_module_name.config(text="-")
        
        self.txt_c_files.config(state=tk.NORMAL)
        self.txt_c_files.delete(1.0, tk.END)
        self.txt_c_files.config(state=tk.DISABLED)
        
        self.txt_h_files.config(state=tk.NORMAL)
        self.txt_h_files.delete(1.0, tk.END)
        self.txt_h_files.config(state=tk.DISABLED)
        
        self.entry_tags.delete(0, tk.END)
        self.txt_desc.delete(1.0, tk.END)

    def save_current_module(self):
        if not self.current_selected_node:
            messagebox.showwarning("警告", "请先在左侧选择一个模块！")
            return

        category_name, module_name = self.current_selected_node
        
        # 获取用户输入
        new_tags_str = self.entry_tags.get()
        new_desc = self.txt_desc.get(1.0, tk.END).strip()

        # 处理 tags: 按逗号分割，去除首尾空格，去掉空字符串
        new_tags = [tag.strip() for tag in new_tags_str.split(',') if tag.strip()]

        # 更新内存中的字典
        self.registry["tree_nodes"][category_name][module_name]["tags"] = new_tags
        self.registry["tree_nodes"][category_name][module_name]["description"] = new_desc

        # 保存到文件
        try:
            with open(self.json_path, 'w', encoding='utf-8') as f:
                json.dump(self.registry, f, indent=4, ensure_ascii=False)
            messagebox.showinfo("成功", f"模块 '{module_name}' 的信息已保存！")
        except Exception as e:
            messagebox.showerror("保存错误", f"写入 JSON 失败: {e}")

if __name__ == "__main__":
    # 测试时如果没设置环境变量，可以取消注释并设置绝对路径
    # os.environ['GMP_PRO_LOCATION'] = r"D:\Your\Project\Root\Path"
    
    root = tk.Tk()
    app = DevResourceMgrGUI(root)
    root.mainloop()