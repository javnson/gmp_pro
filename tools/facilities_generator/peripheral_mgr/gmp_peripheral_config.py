import os
import json
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path

# Unicode 字符用于模拟复选框
CHECKED = "☑ "
UNCHECKED = "☐ "

class UserConfigGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 工程外设配置器 (Project Configurator)")
        self.root.geometry("800x550")

        # 1. 解析路径
        # 1.1 全局库路径 (读取全部可用外设)
        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION，无法加载驱动库！")
            self.root.destroy()
            return
            
        self.global_dic_path = Path(self.gmp_location) / "tools" / "facilities_generator" / "peripheral_mgr" / "gmp_peripheral_dic.json"
        
        # 1.2 本地工程路径 (保存当前工程的选择结果)
        # 注意：这里使用的是 os.getcwd()，即用户在哪个目录下执行bat，这里就是哪里
        self.local_config_path = Path(os.getcwd()) / "gmp_perpheral_config.json"
        
        # 数据存储
        self.global_registry = {}
        self.selected_modules = set() # 存储选中的模块，格式: "category|module"

        # 2. 初始化流程
        self.load_global_dic()
        self.load_local_config()
        self.build_ui()
        self.refresh_tree()

    def load_global_dic(self):
        if not self.global_dic_path.exists():
            messagebox.showerror("错误", "找不到全局外设字典！请先使用开发者工具扫描并生成。")
            self.root.destroy()
            return
        try:
            with open(self.global_dic_path, 'r', encoding='utf-8') as f:
                self.global_registry = json.load(f)
        except Exception as e:
            messagebox.showerror("错误", f"读取全局字典失败: {e}")

    def load_local_config(self):
        """加载当前工程已经勾选的配置"""
        if self.local_config_path.exists():
            try:
                with open(self.local_config_path, 'r', encoding='utf-8') as f:
                    local_config = json.load(f)
                    # 将列表转换为 Set，方便后续快速查找和去重
                    for item in local_config.get("selected_modules", []):
                        self.selected_modules.add(f"{item['category']}|{item['module']}")
            except Exception as e:
                print(f"警告: 本地配置文件读取失败或格式错误 ({e})，将作为新工程处理。")

    def build_ui(self):
        # 顶部提示栏
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill=tk.X)
        ttk.Label(top_frame, text="当前工程路径:").pack(side=tk.LEFT)
        ttk.Label(top_frame, text=str(os.getcwd()), foreground="blue").pack(side=tk.LEFT, padx=5)

        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # ================= 左侧：带复选框的树状图 =================
        left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(left_frame, weight=1)

        ttk.Label(left_frame, text="可用外设库 (点击方框勾选):").pack(anchor=tk.W, pady=(0, 5))

        tree_scroll = ttk.Scrollbar(left_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree = ttk.Treeview(left_frame, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)
        
        self.tree.heading("#0", text=" 模块列表", anchor=tk.W)
        
        # 绑定点击事件，实现复选框切换和详情预览
        self.tree.bind("<ButtonRelease-1>", self.on_tree_click)
        self.tree.bind("<<TreeviewSelect>>", self.show_details)

        # ================= 右侧：只读详情面板 =================
        right_frame = ttk.Frame(self.paned_window, padding=10)
        self.paned_window.add(right_frame, weight=1)

        ttk.Label(right_frame, text="模块名称:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.lbl_name = ttk.Label(right_frame, font=("Helvetica", 10, "bold"), text="-")
        self.lbl_name.grid(row=0, column=1, sticky=tk.W, pady=5)

        ttk.Label(right_frame, text="模块描述:").grid(row=1, column=0, sticky=tk.NW, pady=5)
        self.txt_desc = tk.Text(right_frame, height=4, width=30, state=tk.DISABLED, bg="#f0f0f0")
        self.txt_desc.grid(row=1, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="提供文件:").grid(row=2, column=0, sticky=tk.NW, pady=5)
        self.txt_files = tk.Text(right_frame, height=6, width=30, state=tk.DISABLED, bg="#f0f0f0")
        self.txt_files.grid(row=2, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="分类标签:").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.lbl_tags = ttk.Label(right_frame, text="-")
        self.lbl_tags.grid(row=3, column=1, sticky=tk.W, pady=5)

        right_frame.rowconfigure(4, weight=1)
        right_frame.columnconfigure(1, weight=1)

        # ================= 底部：保存按钮 =================
        bottom_frame = ttk.Frame(self.root, padding=10)
        bottom_frame.pack(fill=tk.X)
        
        self.btn_save = ttk.Button(bottom_frame, text="💾 保存并应用当前工程配置", command=self.save_local_config)
        self.btn_save.pack(side=tk.RIGHT, ipadx=10, ipady=5)

    def refresh_tree(self):
        for item in self.tree.get_children():
            self.tree.delete(item)

        tree_nodes = self.global_registry.get("tree_nodes", {})
        
        for category_name, modules in tree_nodes.items():
            cat_id = f"CAT|{category_name}"
            # 插入分类节点，默认展开
            self.tree.insert("", tk.END, iid=cat_id, text=category_name, open=True)
            
            for module_name, module_data in modules.items():
                if module_data.get("type") == "module":
                    node_id = f"MOD|{category_name}|{module_name}"
                    
                    # 判断该模块是否在当前工程的选中列表中
                    is_selected = node_id.replace("MOD|", "") in self.selected_modules
                    prefix = CHECKED if is_selected else UNCHECKED
                    
                    self.tree.insert(cat_id, tk.END, iid=node_id, text=f"{prefix}{module_name}")

    def on_tree_click(self, event):
        """处理点击事件，模拟复选框的勾选与取消"""
        region = self.tree.identify("region", event.x, event.y)
        if region == "cell" or region == "tree":
            item_id = self.tree.focus()
            if not item_id or not item_id.startswith("MOD|"):
                return
                
            _, category_name, module_name = item_id.split('|')
            module_key = f"{category_name}|{module_name}"
            
            # 切换选中状态
            if module_key in self.selected_modules:
                self.selected_modules.remove(module_key)
                self.tree.item(item_id, text=f"{UNCHECKED}{module_name}")
            else:
                self.selected_modules.add(module_key)
                self.tree.item(item_id, text=f"{CHECKED}{module_name}")

    def show_details(self, event):
        """在右侧显示只读的模块详情"""
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

        files = module_data.get("c_files", []) + module_data.get("h_files", [])
        self.txt_files.config(state=tk.NORMAL)
        self.txt_files.delete(1.0, tk.END)
        self.txt_files.insert(tk.END, "\n".join(files))
        self.txt_files.config(state=tk.DISABLED)

    def clear_details(self):
        self.lbl_name.config(text="-")
        self.lbl_tags.config(text="-")
        self.txt_desc.config(state=tk.NORMAL)
        self.txt_desc.delete(1.0, tk.END)
        self.txt_desc.config(state=tk.DISABLED)
        self.txt_files.config(state=tk.NORMAL)
        self.txt_files.delete(1.0, tk.END)
        self.txt_files.config(state=tk.DISABLED)

    def save_local_config(self):
        """将用户的选择保存为当前目录下的 gmp_perpheral_config.json"""
        config_data = {
            "selected_modules": []
        }
        
        for module_key in self.selected_modules:
            cat, mod = module_key.split('|')
            config_data["selected_modules"].append({
                "category": cat,
                "module": mod
            })
            
        try:
            with open(self.local_config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
            messagebox.showinfo("保存成功", f"工程配置已保存至:\n{self.local_config_path}\n\n接下来您可以运行生成脚本进行代码同步！")
            self.root.destroy() # 保存后自动关闭窗口，提升用户体验
        except Exception as e:
            messagebox.showerror("保存失败", f"无法写入配置文件: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = UserConfigGUI(root)
    root.mainloop()