#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

using json = nlohmann::json;
using namespace std;

// 内部结构：用于扁平化存储操作
struct FlatOp
{
    int mvme_idx;
    int issue_tick;
    string comment;
    json inputs;
    json outputs;
};

class PipelineGenerator
{
  public:
    PipelineGenerator(string json_path)
    {
        ifstream f(json_path);
        if (!f.is_open())
        {
            cerr << "Error: Cannot open JSON file: " << json_path << endl;
            exit(1);
        }
        config = json::parse(f);

        // 读取关键配置
        module_name = config.value("module_name", "mvme_pipeline_top");
        mvme_module_type = config["config"].value("mvme_module", "mvme_35_4ch_std");
        latency = config["config"]["mvme_latency"];
        turn_cycles = config["config"]["turn_cycles"];

        // 预处理流水线数据：解析 Tick 并扁平化
        parse_pipeline();

        // 计算流水线总深度
        int max_tick = 0;
        for (const auto& item : ops_by_tick)
        {
            if (item.first > max_tick)
                max_tick = item.first;
        }
        total_cycle_count = max_tick + latency + 1; // +1 buffer safety
    }

    void generate(string output_path)
    {
        ofstream out(output_path);
        if (!out.is_open())
        {
            cerr << "Error: Cannot write to file: " << output_path << endl;
            exit(1);
        }

        gen_header(out);
        gen_ports(out);
        gen_internal_signals(out);
        gen_shadow_logic(out);
        gen_mvme_instances(out);
        gen_fsm_control(out);
        gen_mux_logic(out);
        gen_latch_logic(out);
        gen_footer(out);

        cout << "Success: Generated " << output_path << endl;
        cout << "Config: Module=" << mvme_module_type << ", Latency=" << latency << ", Turn=" << turn_cycles << endl;
        cout << "Max PC Count: " << total_cycle_count << endl;
    }

  private:
    json config;
    string module_name;
    string mvme_module_type;
    int latency;
    int turn_cycles;
    int total_cycle_count;

    // 核心数据结构：按 Tick 索引的操作列表
    // Key: Tick, Value: List of ops starting at this tick
    map<int, vector<FlatOp>> ops_by_tick;

    map<string, string> output_map = {{"total", "out_total"}, {"mac1", "out_ab_cd"}, {"mac2", "out_ef_gh"},
                                      {"p1", "out_p0"},       {"p2", "out_p1"},      {"p3", "out_p2"},
                                      {"p4", "out_p3"}};

    // --- 解析工具函数 ---

    // 解析 Tick：支持 int 和 "TURN+X" 字符串
    int resolve_tick(const json& j_tick)
    {
        if (j_tick.is_number())
        {
            return j_tick.get<int>();
        }
        else if (j_tick.is_string())
        {
            string s = j_tick.get<string>();
            // 简单的解析逻辑：替换 TURN 后计算
            // 查找 "TURN"
            size_t pos = s.find("TURN");
            if (pos != string::npos)
            {
                // 替换为数值
                s.replace(pos, 4, to_string(turn_cycles));
            }

            // 处理 "A+B" 格式 (非常简易的解析器)
            int result = 0;
            stringstream ss(s);
            string segment;
            while (getline(ss, segment, '+'))
            {
                // 去除空格
                segment.erase(remove(segment.begin(), segment.end(), ' '), segment.end());
                if (!segment.empty())
                {
                    result += stoi(segment);
                }
            }
            return result;
        }
        return 0;
    }

    // 扁平化处理
    void parse_pipeline()
    {
        if (!config.contains("pipeline_sequence"))
            return;

        for (const auto& group : config["pipeline_sequence"])
        {
            int idx = group["mvme_idx"];
            for (const auto& op : group["ops"])
            {
                FlatOp flat_op;
                flat_op.mvme_idx = idx;
                flat_op.issue_tick = resolve_tick(op["tick"]);
                flat_op.inputs = op["inputs"];
                flat_op.outputs = op.value("outputs", json::object());
                flat_op.comment = op.value("comment", "");

                ops_by_tick[flat_op.issue_tick].push_back(flat_op);
            }
        }
    }

    // --- 代码生成函数 ---

    void gen_header(ofstream& out)
    {
        out << "`timescale 1ns / 1ps\n\n";
        out << "module " << module_name << " (\n";
        out << "    input  wire                 clk,\n";
        out << "    input  wire                 rst,\n";
        out << "    input  wire                 enable,\n";
        out << "    input  wire                 trigger,\n";
        out << "    input  wire                 load,\n";
        out << "    input  wire                 update_param,\n";
        out << "    output reg                  param_ready,\n";
        out << "    output reg                  valid,\n\n";
    }

    void gen_ports(ofstream& out)
    {
        out << "    // --- Parameters ---\n";
        for (const auto& p : config["ports"]["parameters"])
        {
            out << "    input  wire signed [34:0] " << p.get<string>() << ",\n";
        }
        out << "\n    // --- State Inputs (Load values) ---\n";
        for (const auto& s : config["ports"]["states_in"])
        {
            out << "    input  wire signed [34:0] " << s.get<string>() << ",\n";
        }
        out << "\n    // --- State Outputs ---\n";
        auto states_out = config["ports"]["states_out"];
        for (size_t i = 0; i < states_out.size(); ++i)
        {
            out << "    output reg  signed [34:0] " << states_out[i].get<string>();
            if (i < states_out.size() - 1)
                out << ",\n";
            else
                out << "\n";
        }
        out << ");\n\n";
    }

    void gen_internal_signals(ofstream& out)
    {
        out << "    // --- Internal Signals ---\n";
        // Shadows
        for (const auto& p : config["ports"]["parameters"])
        {
            string name = p.get<string>();
            out << "    reg signed [34:0] " << name << "_shadow;\n";
            out << "    reg signed [34:0] " << name << "_active;\n";
        }
        // Internal Wires
        for (const auto& w : config["ports"]["internal_wires"])
        {
            out << "    reg signed [34:0] " << w.get<string>() << ";\n";
        }

        // MVME MUX signals
        int mvme_count = config["config"]["mvme_count"];
        for (int i = 0; i < mvme_count; ++i)
        {
            out << "\n    // MVME_" << i << " Interface\n";
            out << "    reg signed [34:0] mvme" << i << "_a, mvme" << i << "_b;\n";
            out << "    reg signed [34:0] mvme" << i << "_c, mvme" << i << "_d;\n";
            out << "    reg signed [34:0] mvme" << i << "_e, mvme" << i << "_f;\n";
            out << "    reg signed [34:0] mvme" << i << "_g, mvme" << i << "_h;\n";
            out << "    wire signed [34:0] mvme" << i << "_out_total;\n";
            out << "    wire signed [34:0] mvme" << i << "_out_ab_cd, mvme" << i << "_out_ef_gh;\n";
            out << "    wire signed [34:0] mvme" << i << "_out_p0, mvme" << i << "_out_p1;\n";
            out << "    wire signed [34:0] mvme" << i << "_out_p2, mvme" << i << "_out_p3;\n";
            out << "    wire               mvme" << i << "_overload;\n";
        }

        // Control Signals
        out << "\n    // Pipeline Control\n";
        out << "    reg [15:0] pc_cnt; // Pipeline Counter\n";
        out << "    reg        running;\n";
        out << "    reg        trigger_d;\n";
        out << "    wire       trigger_pulse = trigger & ~trigger_d;\n";
        out << "    localparam MAX_PC = " << total_cycle_count << ";\n\n";
    }

    void gen_shadow_logic(ofstream& out)
    {
        out << "    // --- Parameter Shadow Logic ---\n";
        out << "    always @(posedge clk) begin\n";
        out << "        if (rst) begin\n";
        out << "            param_ready <= 0;\n";
        for (const auto& p : config["ports"]["parameters"])
        {
            string name = p.get<string>();
            out << "            " << name << "_shadow <= 0; " << name << "_active <= 0;\n";
        }
        out << "        end else begin\n";
        out << "            param_ready <= 0;\n";
        // Shadow update
        for (const auto& p : config["ports"]["parameters"])
        {
            string name = p.get<string>();
            out << "            " << name << "_shadow <= " << name << ";\n";
        }
        // Active update (Safe when not running)
        out << "            if (update_param && !running) begin\n";
        for (const auto& p : config["ports"]["parameters"])
        {
            string name = p.get<string>();
            out << "                " << name << "_active <= " << name << "_shadow;\n";
        }
        out << "                param_ready <= 1;\n";
        out << "            end\n";
        out << "        end\n";
        out << "    end\n\n";
    }

    void gen_mvme_instances(ofstream& out)
    {
        int count = config["config"]["mvme_count"];
        out << "    // --- MVME Instantiation (" << mvme_module_type << ") ---\n";
        for (int i = 0; i < count; ++i)
        {
            out << "    " << mvme_module_type << " u_mvme_" << i << " (\n";
            out << "        .clk(clk), .rst(rst),\n";
            out << "        .a(mvme" << i << "_a), .b(mvme" << i << "_b),\n";
            out << "        .c(mvme" << i << "_c), .d(mvme" << i << "_d),\n";
            out << "        .e(mvme" << i << "_e), .f(mvme" << i << "_f),\n";
            out << "        .g(mvme" << i << "_g), .h(mvme" << i << "_h),\n";
            out << "        .out_total(mvme" << i << "_out_total),\n";
            out << "        .out_ab_cd(mvme" << i << "_out_ab_cd),\n";
            out << "        .out_ef_gh(mvme" << i << "_out_ef_gh),\n";
            out << "        .out_p0(mvme" << i << "_out_p0), .out_p1(mvme" << i << "_out_p1),\n";
            out << "        .out_p2(mvme" << i << "_out_p2), .out_p3(mvme" << i << "_out_p3),\n";
            out << "        .overload(mvme" << i << "_overload)\n";
            out << "    );\n\n";
        }
    }

    void gen_fsm_control(ofstream& out)
    {
        out << "    // --- Pipeline Controller ---\n";
        out << "    always @(posedge clk) begin\n";
        out << "        trigger_d <= trigger;\n";
        out << "        if (rst || !enable) begin\n";
        out << "            pc_cnt <= 0;\n";
        out << "            running <= 0;\n";
        out << "            valid <= 0;\n";
        out << "        end else begin\n";
        out << "            valid <= 0;\n";
        out << "            \n";
        out << "            if (!running) begin\n";
        out << "                pc_cnt <= 0;\n";
        out << "                if (trigger || trigger_pulse) begin\n";
        out << "                    running <= 1;\n";
        out << "                end\n";
        out << "            end else begin\n";
        out << "                if (pc_cnt < MAX_PC) begin\n";
        out << "                    pc_cnt <= pc_cnt + 1;\n";
        out << "                end else begin\n";
        out << "                    valid <= 1;\n";
        out << "                    if (trigger) pc_cnt <= 0;\n";
        out << "                    else running <= 0;\n";
        out << "                end\n";
        out << "            end\n";
        out << "        end\n";
        out << "    end\n\n";
    }

    void gen_mux_logic(ofstream& out)
    {
        out << "    // --- MUX Logic (Input Scheduling) ---\n";
        out << "    always @(*) begin\n";
        // Defaults
        int count = config["config"]["mvme_count"];
        for (int i = 0; i < count; ++i)
        {
            out << "        {mvme" << i << "_a, mvme" << i << "_b, mvme" << i << "_c, mvme" << i << "_d} = 0;\n";
            out << "        {mvme" << i << "_e, mvme" << i << "_f, mvme" << i << "_g, mvme" << i << "_h} = 0;\n";
        }

        out << "\n        case (pc_cnt)\n";

        for (auto const& [tick, ops] : ops_by_tick)
        {
            out << "            " << tick << ": begin\n";
            for (const auto& op : ops)
            {
                int idx = op.mvme_idx;
                if (!op.comment.empty())
                    out << "                // " << op.comment << "\n";

                for (char c = 'a'; c <= 'h'; ++c)
                {
                    string key(1, c);
                    if (!op.inputs.contains(key))
                        continue;

                    string val = op.inputs[key].get<string>();

                    // Check parameter
                    bool is_param = false;
                    for (const auto& p : config["ports"]["parameters"])
                    {
                        if (p.get<string>() == val)
                            is_param = true;
                    }
                    if (is_param)
                        val += "_active";

                    out << "                mvme" << idx << "_" << c << " = " << val << ";\n";
                }
            }
            out << "            end\n";
        }
        out << "        endcase\n";
        out << "    end\n\n";
    }

    void gen_latch_logic(ofstream& out)
    {
        out << "    // --- Latch Logic (Output Capturing) ---\n";
        out << "    always @(posedge clk) begin\n";
        out << "        if (rst) begin\n";
        for (const auto& s : config["ports"]["states_out"])
            out << "            " << s.get<string>() << " <= 0;\n";
        for (const auto& w : config["ports"]["internal_wires"])
            out << "            " << w.get<string>() << " <= 0;\n";

        // Load Logic: 当处于 enable 但非 running，且 load 信号有效时，载入初始值
        out << "        end else if (enable && !running && load && !trigger) begin\n";
        for (size_t i = 0; i < config["ports"]["states_out"].size(); ++i)
        {
            string s_out = config["ports"]["states_out"][i].get<string>();
            string s_in = config["ports"]["states_in"][i].get<string>();
            out << "            " << s_out << " <= " << s_in << ";\n";
        }

        out << "        end else if (running) begin\n";
        out << "            case (pc_cnt)\n";

        // 生成 Latch 逻辑：Tick + Latency
        // 重新遍历 ops_by_tick 并映射到 capture time
        map<int, vector<FlatOp>> capture_map;
        for (auto const& [tick, ops] : ops_by_tick)
        {
            int cap_time = tick + latency;
            for (const auto& op : ops)
            {
                if (!op.outputs.empty())
                    capture_map[cap_time].push_back(op);
            }
        }

        for (auto const& [time, ops] : capture_map)
        {
            out << "                " << time << ": begin\n";
            for (const auto& op : ops)
            {
                int idx = op.mvme_idx;
                if (!op.comment.empty())
                    out << "                    // Capture for: " << op.comment << "\n";

                for (auto it = op.outputs.begin(); it != op.outputs.end(); ++it)
                {
                    string type = it.key();
                    string target = it.value();

                    if (target != "null" && output_map.count(type))
                    {
                        out << "                    " << target << " <= mvme" << idx << "_" << output_map[type]
                            << ";\n";
                    }
                }
            }
            out << "                end\n";
        }

        out << "            endcase\n";
        out << "        end\n";
        out << "    end\n";
    }

    void gen_footer(ofstream& out)
    {
        out << "\nendmodule\n";
    }
};

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        cerr << "Usage: ./generator_v2 <config.json> <output.v>" << endl;
        return 1;
    }
    PipelineGenerator gen(argv[1]);
    gen.generate(argv[2]);
    return 0;
}
