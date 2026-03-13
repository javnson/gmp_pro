import os
import json
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path

# 导入我们的核心引擎
import core_sync_forward_v2 as core_sync_forward
import core_sync_reverse

CHECKED = "☑ "
UNCHECKED = "☐ "

class UserConfigGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 工程外设配置器 V3 (带汇总与双向同步)")
        self.root.geometry("900x650")

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

        # === 右侧：详情面板 ===
        right_frame = ttk.Frame(self.paned_window, padding=10)
        self.paned_window.add(right_frame, weight=1)

        ttk.Label(right_frame, text="节点名称:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.lbl_name = ttk.Label(right_frame, font=("Helvetica", 10, "bold"), text="-")
        self.lbl_name.grid(row=0, column=1, sticky=tk.W, pady=5)

        ttk.Label(right_frame, text="描述信息:").grid(row=1, column=0, sticky=tk.NW, pady=5)
        self.txt_desc = tk.Text(right_frame, height=3, width=35, state=tk.DISABLED, bg="#f0f0f0")
        self.txt_desc.grid(row=1, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="参与编译的源文件 (.c):\n(将复制到工程中)").grid(row=2, column=0, sticky=tk.NW, pady=5)
        self.txt_c_files = tk.Text(right_frame, height=5, width=35, state=tk.DISABLED, bg="#f0f0f0", foreground="blue")
        self.txt_c_files.grid(row=2, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="需要引用的头文件 (.h):\n(将写入 .inl 文件)").grid(row=3, column=0, sticky=tk.NW, pady=5)
        self.txt_h_files = tk.Text(right_frame, height=5, width=35, state=tk.DISABLED, bg="#f0f0f0", foreground="green")
        self.txt_h_files.grid(row=3, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="分类标签:").grid(row=4, column=0, sticky=tk.W, pady=5)
        self.lbl_tags = ttk.Label(right_frame, text="-")
        self.lbl_tags.grid(row=4, column=1, sticky=tk.W, pady=5)

        right_frame.rowconfigure(5, weight=1)
        right_frame.columnconfigure(1, weight=1)

        # === 底部：操作按钮 ===
        bottom_frame = ttk.Frame(self.root, padding=10)
        bottom_frame.pack(fill=tk.X)
        
        # 反向同步按钮 (靠左，使用危险警告色提示)
        self.btn_reverse = ttk.Button(bottom_frame, text="🔙 反向同步检查 (本地修改 -> 库)", command=self.run_reverse_sync)
        self.btn_reverse.pack(side=tk.LEFT, ipadx=10, ipady=5)

        # 正向操作按钮 (靠右)
        self.btn_sync = ttk.Button(bottom_frame, text="🚀 保存并正向同步 (库 -> 本地)", command=self.save_and_sync)
        self.btn_sync.pack(side=tk.RIGHT, ipadx=10, ipady=5, padx=5)
        
        self.btn_save_only = ttk.Button(bottom_frame, text="💾 仅保存配置", command=lambda: self.save_local_config(show_msg=True))
        self.btn_save_only.pack(side=tk.RIGHT, ipadx=10, ipady=5)

    def refresh_tree(self):
        for item in self.tree.get_children(): self.tree.delete(item)
        
        # --- 新增：顶部汇总节点 ---
        self.tree.insert("", tk.END, iid="SUMMARY_NODE", text="📊 当前已选配置汇总", open=True)
        
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
                    
        # 默认选中汇总节点
        self.tree.selection_set("SUMMARY_NODE")
        self.show_summary()

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
            
            # 如果当前恰好显示的是汇总页面，点击复选框时实时刷新汇总数据
            if self.tree.selection() and self.tree.selection()[0] == "SUMMARY_NODE":
                self.show_summary()

    def show_details(self, event):
        selected_items = self.tree.selection()
        if not selected_items: return
        node_id = selected_items[0]
        
        # 判断如果是汇总节点
        if node_id == "SUMMARY_NODE":
            self.show_summary()
            return
            
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

        self.txt_c_files.config(state=tk.NORMAL)
        self.txt_c_files.delete(1.0, tk.END)
        self.txt_c_files.insert(tk.END, "\n".join(module_data.get("c_files", [])))
        self.txt_c_files.config(state=tk.DISABLED)

        self.txt_h_files.config(state=tk.NORMAL)
        self.txt_h_files.delete(1.0, tk.END)
        self.txt_h_files.insert(tk.END, "\n".join(module_data.get("h_files", [])))
        self.txt_h_files.config(state=tk.DISABLED)

    def show_summary(self):
        """动态计算并展示汇总信息"""
        total_c = []
        total_h = []
        
        for module_key in self.selected_modules:
            cat, mod = module_key.split('|')
            try:
                mod_data = self.global_registry["tree_nodes"][cat][mod]
                total_c.extend(mod_data.get("c_files", []))
                total_h.extend(mod_data.get("h_files", []))
            except KeyError:
                continue

        # 去重并排序
        total_c = sorted(list(set(total_c)))
        total_h = sorted(list(set(total_h)))

        self.lbl_name.config(text="全局配置汇总视图")
        self.lbl_tags.config(text="ALL")

        self.txt_desc.config(state=tk.NORMAL)
        self.txt_desc.delete(1.0, tk.END)
        self.txt_desc.insert(tk.END, f"您当前共选中了 {len(self.selected_modules)} 个外设模块。\n\n"
                                     f"包含 .c 源文件: {len(total_c)} 个\n"
                                     f"包含 .h 头文件: {len(total_h)} 个")
        self.txt_desc.config(state=tk.DISABLED)

        self.txt_c_files.config(state=tk.NORMAL)
        self.txt_c_files.delete(1.0, tk.END)
        self.txt_c_files.insert(tk.END, "\n".join(total_c) if total_c else "(暂无选中的源文件)")
        self.txt_c_files.config(state=tk.DISABLED)

        self.txt_h_files.config(state=tk.NORMAL)
        self.txt_h_files.delete(1.0, tk.END)
        self.txt_h_files.insert(tk.END, "\n".join(total_h) if total_h else "(暂无选中的头文件)")
        self.txt_h_files.config(state=tk.DISABLED)

    def clear_details(self):
        self.lbl_name.config(text="-")
        self.lbl_tags.config(text="-")
        for txt in [self.txt_desc, self.txt_c_files, self.txt_h_files]:
            txt.config(state=tk.NORMAL)
            txt.delete(1.0, tk.END)
            txt.config(state=tk.DISABLED)

    def save_local_config(self, show_msg=False):
        config_data = {"selected_modules": []}
        for module_key in self.selected_modules:
            cat, mod = module_key.split('|')
            config_data["selected_modules"].append({"category": cat, "module": mod})
        try:
            with open(self.local_config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
            if show_msg:
                messagebox.showinfo("保存成功", "工程配置已更新，但不关闭窗口。")
            return True
        except Exception as e:
            messagebox.showerror("保存失败", f"无法写入配置文件: {e}")
            return False

    def save_and_sync(self):
        if not self.save_local_config(): return
        # 调用正向同步，完成后弹出提示，不再 destroy root
        success, message = core_sync_forward.run_forward_sync()
        if success:
            messagebox.showinfo("同步成功", message)
        else:
            messagebox.showerror("同步失败", message)
            
    def run_reverse_sync(self):
        # 启动反向同步的高级 Diff 窗口
        core_sync_reverse.run_reverse_sync_gui(self.root, self.global_registry, self.selected_modules)

if __name__ == "__main__":
    root = tk.Tk()
    app = UserConfigGUI(root)
    root.mainloop()