#ifndef FAUBTB_PERF_H
#define FAUBTB_PERF_H

#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>

/*
 * 硬件编号及含义必须与 faubtb_event_deltas 的顺序保持一致。
 * “新预测”指在途表保存的候选预测，不代表实际实施了旁路。
 */
#define FAUBTB_EVENT_LIST(X) \
 X(64, cfi_total, "纳入统计的实际CFI总数") \
 X(65, cfi_hit, "上述CFI中预测时FAuBTB命中的数量") \
 X(66, cfi_error, "上述CFI中按当前总错误判定发生预测错误的数量") \
 X(67, cfi_duplicate, "更新时其他way存在相同tag的CFI事件数，每个slot最多计一次") \
 X(68, br_hit, "条件分支预测时FAuBTB命中的数量") \
 X(69, br_miss_not_taken, "条件分支未命中且实际不跳转的数量") \
 X(70, br_miss_taken, "条件分支未命中且实际跳转的数量") \
 X(71, br_direction_correct, "条件分支命中且预测方向正确的数量") \
 X(72, br_direction_incorrect, "条件分支命中但预测方向错误的数量") \
 X(73, br_target_correct, "条件分支命中且方向正确并实际跳转时目标正确的数量") \
 X(74, br_target_incorrect, "条件分支命中且方向正确并实际跳转时目标错误的数量") \
 X(75, jal_hit, "JAL预测时FAuBTB命中的数量") \
 X(76, jal_miss, "JAL预测时FAuBTB未命中的数量") \
 X(77, jal_target_correct, "JAL命中且预测目标正确的数量，仅比较目标") \
 X(78, jal_target_incorrect, "JAL命中但预测目标错误的数量，仅比较目标") \
 X(79, jalr_hit, "JALR预测时FAuBTB命中的数量") \
 X(80, jalr_miss, "JALR预测时FAuBTB未命中的数量") \
 X(81, jalr_target_correct, "JALR命中且预测目标正确的数量，仅比较目标") \
 X(82, jalr_target_incorrect, "JALR命中但预测目标错误的数量，仅比较目标") \
 X(83, stale_cfi, "有效更新中对应在途记录被标记为stale的CFI数量") \
 X(84, stale_direction_diff_old_correct, "stale样本中旧新方向不同且旧方向正确的数量") \
 X(85, stale_direction_diff_new_correct, "stale样本中旧新方向不同且候选新方向正确的数量") \
 X(86, stale_target_diff_old_correct, "stale样本中双方预测跳转且实际跳转并且目标不同，旧目标正确的数量") \
 X(87, stale_target_diff_new_correct, "stale样本中双方预测跳转且实际跳转并且目标不同，候选新目标正确的数量") \
 X(88, stale_target_diff_both_wrong, "stale样本中双方预测跳转且实际跳转并且目标不同，两个目标都错误的数量") \
 X(89, stale_prediction_new_correct_old_incorrect, "stale样本中按当前完整预测判定候选新预测正确而旧预测错误的数量") \
 X(90, stale_prediction_old_correct_new_incorrect, "stale样本中按当前完整预测判定旧预测正确而候选新预测错误的数量")

struct FAuBTBEventInfo {
    unsigned counter_id;
    const char *name;
    const char *meaning;
};

static const FAuBTBEventInfo faubtb_event_info[] = {
#define FAUBTB_INFO(id, name, meaning) {id, #name, meaning},
    FAUBTB_EVENT_LIST(FAUBTB_INFO)
#undef FAUBTB_INFO
};

static_assert(
    sizeof(faubtb_event_info) / sizeof(faubtb_event_info[0]) == 27,
    "FAuBTB event count must be 27"
);

static inline void faubtb_dump(
    FILE *logfile, uint64_t sampletimes, uint64_t exitpc)
{
    uint64_t values[27];

    /*
     * BOOM-Stop 自定义读取指令：
     * tag = 128 + 硬件计数器编号。
     * 读取结果写入指令 rs1 字段指定的通用寄存器。
     */
#define FAUBTB_READ(id, name, meaning) \
    __asm__ volatile( \
        "addi x0, %0, %1" \
        : "=r"(values[(id) - 64]) \
        : "i"(128 + (id)) \
        : "memory" \
    );

    FAUBTB_EVENT_LIST(FAUBTB_READ)
#undef FAUBTB_READ

    /* 硬件只读取一次，同一份数值分别输出到日志和终端。 */
    FILE *outputs[2] = {logfile, stdout};

    for (unsigned output = 0; output < 2; ++output) {
        FILE *out = outputs[output];
        if (out == NULL || (output == 1 && logfile == stdout))
            continue;

        fprintf(out,
            "{\"type\":\"faubtb_window\","
            "\"sample\":%" PRIu64 ","
            "\"exitpc\":\"0x%" PRIx64 "\","
            "\"scope\":\"本窗口自上次清零以来，满足进程、特权级和使能过滤，"
            "按有效提交更新到达时累计；不保证等于整个程序总数\"}\n",
            sampletimes, exitpc);

        for (unsigned i = 0; i < 27; ++i) {
            fprintf(out,
                "{\"type\":\"faubtb_counter\","
                "\"sample\":%" PRIu64 ","
                "\"counter\":%u,"
                "\"name\":\"%s\","
                "\"value\":%" PRIu64 ","
                "\"unit\":\"动态CFI事件数\","
                "\"meaning\":\"%s\"}\n",
                sampletimes,
                faubtb_event_info[i].counter_id,
                faubtb_event_info[i].name,
                values[i],
                faubtb_event_info[i].meaning);
        }

        fflush(out);
    }
}

#undef FAUBTB_EVENT_LIST
#endif