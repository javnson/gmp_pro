import os
import json
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path
# 导入我们刚刚写的同步引擎
import core_sync_forward

CHECKED = "☑ "
UNCHECKED = "☐ "

class UserConfigGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 工程外设配置器 V2 (支持一键同步)")
        self.root.geometry("850x600")

        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION")
            self.root.destroy()
            return
            
        self.global_dic_path = Path(self.gmp_location) / "tools" / "facilities_generator" / "peripheral_mgr" / "gmp_peripheral_dic.json"
        self.local_config_path = Path(os.getcwd()) / "gmp_perpheral_config.json"
        
        self.global_registry = {}
        self.selected_modules = set()

        self.load_global_dic()
        self.load_local_config()
        self.build_ui()
        self.refresh_tree()

    def load_global_dic(self):
        if not self.global_dic_path.exists():
            messagebox.showerror("错误", "找不到全局外设字典！")
            self.root.destroy()
            return
        with open(self.global_dic_path, 'r', encoding='utf-8') as f:
            self.global_registry = json.load(f)

    def load_local_config(self):
        if self.local_config_path.exists():
            try:
                with open(self.local_config_path, 'r', encoding='utf-8') as f:
                    local_config = json.load(f)
                    for item in local_config.get("selected_modules", []):
                        self.selected_modules.add(f"{item['category']}|{item['module']}")
            except:
                pass

    def build_ui(self):
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill=tk.X)
        ttk.Label(top_frame, text="当前工程路径:").pack(side=tk.LEFT)
        ttk.Label(top_frame, text=str(os.getcwd()), foreground="blue").pack(side=tk.LEFT, padx=5)

        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # === 左侧：勾选树 ===
        left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(left_frame, weight=1)

        ttk.Label(left_frame, text="可用外设库 (点击方框勾选):").pack(anchor=tk.W, pady=(0, 5))
        tree_scroll = ttk.Scrollbar(left_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree = ttk.Treeview(left_frame, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)
        self.tree.heading("#0", text=" 模块列表", anchor=tk.W)
        self.tree.bind("<ButtonRelease-1>", self.on_tree_click)
        self.tree.bind("<<TreeviewSelect>>", self.show_details)

        # === 右侧：详情面板 (分离 .c 和 .h) ===
        right_frame = ttk.Frame(self.paned_window, padding=10)
        self.paned_window.add(right_frame, weight=1)

        ttk.Label(right_frame, text="模块名称:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.lbl_name = ttk.Label(right_frame, font=("Helvetica", 10, "bold"), text="-")
        self.lbl_name.grid(row=0, column=1, sticky=tk.W, pady=5)

        ttk.Label(right_frame, text="模块描述:").grid(row=1, column=0, sticky=tk.NW, pady=5)
        self.txt_desc = tk.Text(right_frame, height=3, width=35, state=tk.DISABLED, bg="#f0f0f0")
        self.txt_desc.grid(row=1, column=1, sticky=tk.EW, pady=5)

        # 分离的 Source 框
        ttk.Label(right_frame, text="参与编译的源文件 (.c):\n(将复制到工程中)").grid(row=2, column=0, sticky=tk.NW, pady=5)
        self.txt_c_files = tk.Text(right_frame, height=4, width=35, state=tk.DISABLED, bg="#f0f0f0", foreground="blue")
        self.txt_c_files.grid(row=2, column=1, sticky=tk.EW, pady=5)

        # 分离的 Header 框
        ttk.Label(right_frame, text="需要引用的头文件 (.h):\n(将写入 .inl 文件)").grid(row=3, column=0, sticky=tk.NW, pady=5)
        self.txt_h_files = tk.Text(right_frame, height=4, width=35, state=tk.DISABLED, bg="#f0f0f0", foreground="green")
        self.txt_h_files.grid(row=3, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="分类标签:").grid(row=4, column=0, sticky=tk.W, pady=5)
        self.lbl_tags = ttk.Label(right_frame, text="-")
        self.lbl_tags.grid(row=4, column=1, sticky=tk.W, pady=5)

        right_frame.rowconfigure(5, weight=1)
        right_frame.columnconfigure(1, weight=1)

        # === 底部：操作按钮 ===
        bottom_frame = ttk.Frame(self.root, padding=10)
        bottom_frame.pack(fill=tk.X)
        
        # 增加一键同步按钮
        self.btn_sync = ttk.Button(bottom_frame, text="🚀 保存并一键生成/同步代码", command=self.save_and_sync)
        self.btn_sync.pack(side=tk.RIGHT, ipadx=10, ipady=5, padx=5)
        
        self.btn_save_only = ttk.Button(bottom_frame, text="💾 仅保存配置", command=self.save_local_config)
        self.btn_save_only.pack(side=tk.RIGHT, ipadx=10, ipady=5)

    def refresh_tree(self):
        for item in self.tree.get_children(): self.tree.delete(item)
        tree_nodes = self.global_registry.get("tree_nodes", {})
        for category_name, modules in tree_nodes.items():
            cat_id = f"CAT|{category_name}"
            self.tree.insert("", tk.END, iid=cat_id, text=category_name, open=True)
            for module_name, module_data in modules.items():
                if module_data.get("type") == "module":
                    node_id = f"MOD|{category_name}|{module_name}"
                    is_selected = node_id.replace("MOD|", "") in self.selected_modules
                    prefix = CHECKED if is_selected else UNCHECKED
                    self.tree.insert(cat_id, tk.END, iid=node_id, text=f"{prefix}{module_name}")

    def on_tree_click(self, event):
        region = self.tree.identify("region", event.x, event.y)
        if region == "cell" or region == "tree":
            item_id = self.tree.focus()
            if not item_id or not item_id.startswith("MOD|"): return
            _, category_name, module_name = item_id.split('|')
            module_key = f"{category_name}|{module_name}"
            if module_key in self.selected_modules:
                self.selected_modules.remove(module_key)
                self.tree.item(item_id, text=f"{UNCHECKED}{module_name}")
            else:
                self.selected_modules.add(module_key)
                self.tree.item(item_id, text=f"{CHECKED}{module_name}")

    def show_details(self, event):
        selected_items = self.tree.selection()
        if not selected_items: return
        node_id = selected_items[0]
        if node_id.startswith("CAT|"):
            self.clear_details()
            return
        _, category_name, module_name = node_id.split('|')
        module_data = self.global_registry["tree_nodes"][category_name][module_name]

        self.lbl_name.config(text=module_name)
        self.lbl_tags.config(text=", ".join(module_data.get("tags", [])))

        self.txt_desc.config(state=tk.NORMAL)
        self.txt_desc.delete(1.0, tk.END)
        self.txt_desc.insert(tk.END, module_data.get("description", ""))
        self.txt_desc.config(state=tk.DISABLED)

        # 分别填充 .c 和 .h，并且用换行符隔开看起来更清晰
        self.txt_c_files.config(state=tk.NORMAL)
        self.txt_c_files.delete(1.0, tk.END)
        self.txt_c_files.insert(tk.END, "\n".join(module_data.get("c_files", [])))
        self.txt_c_files.config(state=tk.DISABLED)

        self.txt_h_files.config(state=tk.NORMAL)
        self.txt_h_files.delete(1.0, tk.END)
        self.txt_h_files.insert(tk.END, "\n".join(module_data.get("h_files", [])))
        self.txt_h_files.config(state=tk.DISABLED)

    def clear_details(self):
        self.lbl_name.config(text="-")
        self.lbl_tags.config(text="-")
        for txt in [self.txt_desc, self.txt_c_files, self.txt_h_files]:
            txt.config(state=tk.NORMAL)
            txt.delete(1.0, tk.END)
            txt.config(state=tk.DISABLED)

    def save_local_config(self):
        config_data = {"selected_modules": []}
        for module_key in self.selected_modules:
            cat, mod = module_key.split('|')
            config_data["selected_modules"].append({"category": cat, "module": mod})
        try:
            with open(self.local_config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
            return True
        except Exception as e:
            messagebox.showerror("保存失败", f"无法写入配置文件: {e}")
            return False

    def save_and_sync(self):
        # 1. 先保存配置
        if not self.save_local_config():
            return
            
        # 2. 调用同步引擎
        success, message = core_sync_forward.run_forward_sync()
        
        if success:
            messagebox.showinfo("同步成功", message)
            self.root.destroy() # 搞定收工，自动关闭
        else:
            messagebox.showerror("同步失败", message)

if __name__ == "__main__":
    root = tk.Tk()
    app = UserConfigGUI(root)
    root.mainloop()