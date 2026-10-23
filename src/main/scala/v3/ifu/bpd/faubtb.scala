package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._

import boom.v3.common._
import boom.v3.util.{BoomCoreStringPrefix, WrapInc}

import scala.math.min

case class BoomFAMicroBTBParams(
  nWays: Int = 16,
  offsetSz: Int = 13
)


class FAMicroBTBBranchPredictorBank(params: BoomFAMicroBTBParams = BoomFAMicroBTBParams())(implicit p: Parameters) extends BranchPredictorBank()(p)
{
  override val nWays         = params.nWays
  val tagSz         = vaddrBitsExtended - log2Ceil(fetchWidth) - 1
  val offsetSz      = params.offsetSz
  val nWrBypassEntries = 2

  def bimWrite(v: UInt, taken: Bool): UInt = {
    val old_bim_sat_taken  = v === 3.U
    val old_bim_sat_ntaken = v === 0.U
    Mux(old_bim_sat_taken  &&  taken, 3.U,
      Mux(old_bim_sat_ntaken && !taken, 0.U,
      Mux(taken, v + 1.U, v - 1.U)))
  }

  require(isPow2(nWays))

  class MicroBTBEntry extends Bundle {
    val offset   = SInt(offsetSz.W)
  }

  class MicroBTBMeta extends Bundle {
    val is_br = Bool()
    val tag   = UInt(tagSz.W)
    val ctr   = UInt(2.W)
  }

  val inflightDepth = 512
  require(isPow2(inflightDepth))
  val inflightPtrWidth = log2Ceil(inflightDepth)

  class MicroBTBPredictMeta extends Bundle {
    val hits  = Vec(bankWidth, Bool())
    val write_way = UInt(log2Ceil(nWays).W)

    //额外保存的预测时信息
    val pred_ctr = Vec(bankWidth, UInt(2.W))
    val pred_taken = Vec(bankWidth, Bool())
    val pred_target = Vec(bankWidth, UInt(vaddrBitsExtended.W))

     // 本次预测写入在途队列的位置,以及pred_cycle，用于对应的在途表项有没有被覆盖
    val inflight_idx = UInt(inflightPtrWidth.W)
    val pred_cycle = UInt(64.W)
  }

  val s1_meta = Wire(new MicroBTBPredictMeta)
  override val metaSz = s1_meta.asUInt.getWidth

  val meta     = RegInit((0.U).asTypeOf(Vec(nWays, Vec(bankWidth, new MicroBTBMeta))))
  val btb      = Reg(Vec(nWays, Vec(bankWidth, new MicroBTBEntry)))

  val mems = Nil

  val s1_req_tag   = s1_idx

  val s1_resp   = Wire(Vec(bankWidth, Valid(UInt(vaddrBitsExtended.W))))
  val s1_taken  = Wire(Vec(bankWidth, Bool()))
  val s1_is_br  = Wire(Vec(bankWidth, Bool()))
  val s1_is_jal = Wire(Vec(bankWidth, Bool()))

  val s1_hit_ohs = VecInit((0 until bankWidth) map { i =>
    VecInit((0 until nWays) map { w =>
      meta(w)(i).tag === s1_req_tag(tagSz-1,0)
    })
  })
  val s1_hits     = s1_hit_ohs.map { oh => oh.reduce(_||_) }
  val s1_hit_ways = s1_hit_ohs.map { oh => PriorityEncoder(oh) }

  for (w <- 0 until bankWidth) {
    val entry_meta = meta(s1_hit_ways(w))(w)
    s1_resp(w).valid := s1_valid && s1_hits(w)
    s1_resp(w).bits  := (s1_pc.asSInt + (w << 1).S + btb(s1_hit_ways(w))(w).offset).asUInt
    s1_is_br(w)      := s1_resp(w).valid &&  entry_meta.is_br
    s1_is_jal(w)     := s1_resp(w).valid && !entry_meta.is_br
    s1_taken(w)      := !entry_meta.is_br || entry_meta.ctr(1)

    s1_meta.hits(w)     := s1_hits(w)

    s1_meta.pred_ctr(w) := entry_meta.ctr
    s1_meta.pred_taken(w) := s1_hits(w) && s1_taken(w)
    s1_meta.pred_target(w) := s1_resp(w).bits
  }

// ============================================================
// 在途 Fetch Packet 队列
// ============================================================
// F1 预测周期。
val debug_cycle = RegInit(0.U(64.W))
  debug_cycle := debug_cycle + 1.U

class InflightEntry extends Bundle {
  val valid = Bool()
  // Fetch Packet 身份
  val tag        = UInt(tagSz.W)
  val pred_cycle = UInt(64.W)
  val write_way = UInt(log2Ceil(nWays).W)

  // 每个slot是否是基于旧表项的预测,如果是，那应该基于的新表项是什么
  val stale = Vec(bankWidth, Bool())
  val alt_ctr    = Vec(bankWidth, UInt(2.W))
  val alt_taken  = Vec(bankWidth, Bool())
  val alt_target = Vec(bankWidth, UInt(vaddrBitsExtended.W))
}
//创建在途表
val inflightQueue = RegInit(
  0.U.asTypeOf(Vec(inflightDepth, new InflightEntry))
)

//首尾指针
val inflightHead = RegInit(0.U(inflightPtrWidth.W))
val inflightTail = RegInit(0.U(inflightPtrWidth.W))
val inflightEnqueue = s1_valid

    //将在途表的索引写进meta
  s1_meta.inflight_idx := inflightTail
  s1_meta.pred_cycle := debug_cycle

when (inflightEnqueue) {
  inflightQueue(inflightTail).valid := true.B
  inflightQueue(inflightTail).tag := s1_req_tag(tagSz - 1, 0)
  inflightQueue(inflightTail).pred_cycle := debug_cycle
  inflightQueue(inflightTail).write_way := s1_meta.write_way

  for (w <- 0 until bankWidth) {
    // 新入队时还没有发现它使用旧状态
    inflightQueue(inflightTail).stale(w) := false.B
    inflightQueue(inflightTail).alt_ctr(w) := 0.U
    inflightQueue(inflightTail).alt_taken(w) := false.B
    inflightQueue(inflightTail).alt_target(w) := 0.U
  }

  inflightTail := inflightTail + 1.U
}
//---------------------------------------------------------------
  

  val alloc_way = {
    val r_metas = Cat(VecInit(meta.map(e => VecInit(e.map(_.tag)))).asUInt, s1_idx(tagSz-1,0))
    val l = log2Ceil(nWays)
    val nChunks = (r_metas.getWidth + l - 1) / l
    val chunks = (0 until nChunks) map { i =>
      r_metas(min((i+1)*l, r_metas.getWidth)-1, i*l)
    }
    chunks.reduce(_^_)
  }
  //基准版本：FAuBTB命中way，否则alloc_way。
  s1_meta.write_way := Mux(s1_hits.reduce(_||_),
    PriorityEncoder(s1_hit_ohs.map(_.asUInt).reduce(_|_)),
    alloc_way)

//   // 全表查询：不用head/tail，只复用未被提交进度越过的已写入记录。
//   s1_meta.write_way := Mux(
//     s1_hits.reduce(_ || _),
//     PriorityEncoder(s1_hit_ohs.map(_.asUInt).reduce(_ | _)),
//     {
//       val inflightMatches = VecInit((0 until inflightDepth).map { i =>
//         inflightQueue(i).valid &&
//         (!inflightHasCommit ||
//           inflightQueue(i).pred_cycle > inflightLastCommitPredCycle) &&
//         (inflightQueue(i).tag === s1_req_tag(tagSz - 1, 0))
//       }).asUInt
//       Mux(inflightMatches.orR,
//         inflightQueue(PriorityEncoder(inflightMatches)).write_way,
//         alloc_way)
//     }
//   )
  //--------------------------------------------------------------------------

  for (w <- 0 until bankWidth) {
    io.resp.f1(w).predicted_pc := s1_resp(w)
    io.resp.f1(w).is_br        := s1_is_br(w)
    io.resp.f1(w).is_jal       := s1_is_jal(w)
    io.resp.f1(w).taken        := s1_taken(w)

    io.resp.f2(w) := RegNext(io.resp.f1(w))
    io.resp.f3(w) := RegNext(io.resp.f2(w))
  }
  io.f3_meta := RegNext(RegNext(s1_meta.asUInt))

//update
  val s1_update_cfi_idx = s1_update.bits.cfi_idx.bits
  val s1_update_meta    = s1_update.bits.meta.asTypeOf(new MicroBTBPredictMeta)
  val s1_update_write_way = s1_update_meta.write_way

  val max_offset_value = (~(0.U)((offsetSz-1).W)).asSInt
  val min_offset_value = Cat(1.B, (0.U)((offsetSz-1).W)).asSInt
  val new_offset_value = (s1_update.bits.target.asSInt -
    (s1_update.bits.pc + (s1_update.bits.cfi_idx.bits << 1)).asSInt)

  val s1_update_wbtb_data     = Wire(new MicroBTBEntry)
  s1_update_wbtb_data.offset := new_offset_value
  val s1_update_wbtb_mask = (UIntToOH(s1_update_cfi_idx) &
    Fill(bankWidth, s1_update.bits.cfi_idx.valid && s1_update.valid && s1_update.bits.cfi_taken && s1_update.bits.is_commit_update))

  val s1_update_wmeta_mask = ((s1_update_wbtb_mask | s1_update.bits.br_mask) &
    Fill(bankWidth, s1_update.valid && s1_update.bits.is_commit_update))

  // Write the BTB with the target
  when (s1_update.valid && s1_update.bits.cfi_taken && s1_update.bits.cfi_idx.valid && s1_update.bits.is_commit_update) {
    btb(s1_update_write_way)(s1_update_cfi_idx).offset := new_offset_value
  }

  // Write the meta
  for (w <- 0 until bankWidth) {
    when (s1_update.valid && s1_update.bits.is_commit_update &&
      (s1_update.bits.br_mask(w) ||
        (s1_update_cfi_idx === w.U && s1_update.bits.cfi_taken && s1_update.bits.cfi_idx.valid))) {
      val was_taken = (s1_update_cfi_idx === w.U && s1_update.bits.cfi_idx.valid &&
        (s1_update.bits.cfi_taken || s1_update.bits.cfi_is_jal))

      meta(s1_update_write_way)(w).is_br := s1_update.bits.br_mask(w)
      meta(s1_update_write_way)(w).tag   := s1_update_idx
      meta(s1_update_write_way)(w).ctr   := Mux(!(s1_update_meta.hits(w)),
        Mux(was_taken, 3.U, 0.U),
        bimWrite(meta(s1_update_write_way)(w).ctr, was_taken)
      )
    }
  }


//更新阶段，在途表检查基于旧值的预测
// ============================================================
// 1. 根据预测时保存的队列指针，直接找到更新对应的记录，先检查又有没有被覆盖，被覆盖直接断言结束
// ============================================================
val validCommitUpdate = s1_update.valid && s1_update.bits.is_commit_update
val updateQueueIdx = s1_update_meta.inflight_idx

when (validCommitUpdate) {
  // tag检查地址，pred_cycle区分同一地址的不同动态查询。
  val updateEntryMatches =
    inflightQueue(updateQueueIdx).tag === s1_update_idx(tagSz - 1, 0) &&
    inflightQueue(updateQueueIdx).pred_cycle === s1_update_meta.pred_cycle
  assert(updateEntryMatches, "FAuBTB inflight entry identity mismatch: overwritten or wrong pointer")


  inflightHead := updateQueueIdx + 1.U
}
// ============================================================
//2. 往后看在途表检查是否存在基于旧的信息的预测，有的话给他们打上标记
// ============================================================
val shadowNewCtr = VecInit(
  (0 until bankWidth).map { w =>
    val wasTaken =
      s1_update_cfi_idx === w.U && s1_update.bits.cfi_idx.valid &&
      (s1_update.bits.cfi_taken || s1_update.bits.cfi_is_jal)

    Mux( !s1_update_meta.hits(w),
          Mux(wasTaken, 3.U, 0.U),
          bimWrite(
            meta(s1_update_write_way)(w).ctr,
            wasTaken
          )
    )
  }
)
val shadowNewTarget = VecInit((0 until bankWidth).map { w =>
  (s1_update.bits.pc.asSInt + (w << 1).S +
    Mux(s1_update_wbtb_mask(w), new_offset_value(offsetSz - 1, 0).asSInt, btb(s1_update_write_way)(w).offset)).asUInt
})

// 先逐个检查在途表中年轻Fetch Packet的tag
for (i <- 0 until inflightDepth) {
  val queueIdx = i.U(inflightPtrWidth.W)

//这个位置是否位于首尾指针之间，也就是是不是有效区间的表项
  val youngerEntryValid = Mux(
    updateQueueIdx < inflightTail,
    queueIdx > updateQueueIdx &&
      queueIdx < inflightTail,
    queueIdx > updateQueueIdx ||
      queueIdx < inflightTail
  )

//同一个Fetch Packet再次出现在路径上，说明是基于旧值的更新
  when (
    validCommitUpdate &&
    youngerEntryValid &&
    inflightQueue(i).tag === s1_update_idx(tagSz - 1, 0)
  ) {
    for (w <- 0 until bankWidth) {
      //此次更新的几个slot，之前用的一定是旧值
      when (s1_update_wmeta_mask(w)) {
        inflightQueue(i).stale(w) := true.B
        inflightQueue(i).alt_ctr(w) := shadowNewCtr(w)
        inflightQueue(i).alt_taken(w) :=
          Mux(
            s1_update.bits.br_mask(w),
            shadowNewCtr(w)(1),
            true.B
          )
        inflightQueue(i).alt_target(w) := shadowNewTarget(w)
      }
    }
  }
}

    //表里检查完，检查正在查找的是否也有tag重复
    when (inflightEnqueue && validCommitUpdate &&
      s1_req_tag(tagSz - 1, 0) === s1_update_idx(tagSz - 1, 0)) {
      for (w <- 0 until bankWidth) {
       when (s1_update_wmeta_mask(w)) {
        inflightQueue(inflightTail).stale(w):= true.B
        inflightQueue(inflightTail).alt_ctr(w) := shadowNewCtr(w)
        inflightQueue(inflightTail).alt_taken(w) :=
          Mux(
            s1_update.bits.br_mask(w),
            shadowNewCtr(w)(1),
            s1_update_wbtb_mask(w)
          )
        inflightQueue(inflightTail).alt_target(w) := shadowNewTarget(w)
      }
    }
  }
//------------------------------------------------------------------------
  // ------------------------------------------------------------
  // 检查命中率，预测准确率.注意，我们是在正确执行路径上的转移指令基础上检查
  // ------------------------------------------------------------
    val slot_statistics = (0 until bankWidth).map { w =>

    val prediction_valid = s1_update_wmeta_mask(w) && s1_update_meta.hits(w)

  //预测时是否taken,要求预测命中并且taken
    val predicted_taken = prediction_valid && s1_update_meta.pred_taken(w)

  //当前slot的target
    val predicted_target = s1_update_meta.pred_target(w)

    // 当前 slot 是否为 cfi_idx 指向的
    val slot_is_cfi_idx = s1_update.bits.cfi_idx.valid && s1_update.bits.cfi_idx.bits === w.U

    // 转移指令类型：是否为branch，是根据实际执行情况得到的
    val actual_branch = s1_update.bits.br_mask(w)

    // 转移指令类型，是否为jal
    val actual_jal = !actual_branch && slot_is_cfi_idx && s1_update.bits.cfi_is_jal && !s1_update.bits.cfi_is_jalr

    // 转移指令类型，是否为jalr
    val actual_jalr = !actual_branch && slot_is_cfi_idx && s1_update.bits.cfi_is_jalr

    //实际执行时是否taken，只有cfi_idx指向的slot才有可能taken
    val actual_taken = slot_is_cfi_idx && s1_update.bits.cfi_taken

    //比较的时候未命中，但是实际执行为taken,这会导致错误
    val miss_but_taken =
      s1_update_wmeta_mask(w) && !s1_update_meta.hits(w) && actual_taken

    //1.1 BR命中和未命中，若未命中是否taken
    val branch_hit = actual_branch && prediction_valid
    val branch_miss_not_taken =
      actual_branch && s1_update_wmeta_mask(w) && !s1_update_meta.hits(w) && !actual_taken
    val branch_miss_taken =
      actual_branch && s1_update_wmeta_mask(w) && !s1_update_meta.hits(w) && actual_taken

    //1.2 BR 命中后预测方向是否准确。
    val branch_direction_correct =
      branch_hit && (predicted_taken === actual_taken)
    val branch_direction_incorrect =
      branch_hit && (predicted_taken =/= actual_taken)
    
    //1.3 BR 目标是否准确率：要求在命中并且方向正确并且taken时，看目标是否正确
    val branch_target_correct =
      branch_direction_correct && actual_taken && (predicted_target === s1_update.bits.target)
    val branch_target_incorrect =
      branch_direction_correct && actual_taken && (predicted_target =/= s1_update.bits.target)


    //2.1 JAL命中与未命中
    val jal_hit = actual_jal && prediction_valid
    val jal_miss = s1_update_wmeta_mask(w) && actual_jal && !s1_update_meta.hits(w)
    //2.2 JAL 转移目标准确率，要求命中的前提下检查目标准确率,因为假如未命中，jal和jalr一定是预测错误
    val jal_target_correct = jal_hit && (predicted_target === s1_update.bits.target)
    val jal_target_incorrect = jal_hit && (predicted_target =/= s1_update.bits.target)

    //3.1 JALR命中与未命中
    val jalr_hit =  actual_jalr && prediction_valid 
    val jalr_miss = s1_update_wmeta_mask(w) && actual_jalr && !s1_update_meta.hits(w)
    //3.2 JALR 目标准确率
    val jalr_target_correct =jalr_hit && (predicted_target === s1_update.bits.target)
    val jalr_target_incorrect =jalr_hit && (predicted_target =/= s1_update.bits.target)

    //统计到底是否预测错误
    val mispredicted_cfi_count =  miss_but_taken ||
      (actual_branch && prediction_valid && (predicted_taken =/= actual_taken)) ||
      (actual_branch && prediction_valid && actual_taken && (predicted_taken === actual_taken) && (predicted_target =/= s1_update.bits.target)) ||
      (actual_jal && prediction_valid && (predicted_target =/= s1_update.bits.target)) ||
      (actual_jalr && prediction_valid && (predicted_target =/= s1_update.bits.target))
      


    // 每个 slot 返回六组，每组统一为四元组。
    Seq(
      (mispredicted_cfi_count, false.B, false.B, false.B),
      (branch_hit, branch_miss_not_taken, branch_miss_taken, false.B),
      (branch_direction_correct, branch_direction_incorrect, false.B, false.B),
      (branch_target_correct, branch_target_incorrect, false.B, false.B),
      (jal_hit, jal_miss, jal_target_correct, jal_target_incorrect),
      (jalr_hit, jalr_miss, jalr_target_correct, jalr_target_incorrect)
    )
  }

  val actual_cfi_count = PopCount(s1_update_wmeta_mask)

//本次写入的 way 一定会保存这个 tag，所以只要其他 way 已有相同 tag，写入后就会重复。
  val duplicate_cfi_count = PopCount(VecInit((0 until bankWidth).map { w =>
    s1_update_wmeta_mask(w) && VecInit((0 until nWays).map { way =>
      (way.U =/= s1_update_write_way) &&
      (meta(way)(w).tag === s1_update_idx(tagSz - 1, 0))
    }).asUInt.orR
  }))

  val covered_cfi_count =
    PopCount(
      s1_update_wmeta_mask &
      s1_update_meta.hits.asUInt 
    )
  val mispredicted_cfi_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(0)._1)))


  val stale_entry = inflightQueue(updateQueueIdx)
  //将本次更新的slot和stale标记的slot取交集，即为预测时使用的是旧值
  val stale_cfi_mask = s1_update_wmeta_mask & stale_entry.stale.asUInt
  val stale_cfi_count = PopCount(stale_cfi_mask)


  
  // 统计基于旧值预测和基于新值预测是否相同，若不同谁是对的
  val stale_prediction_statistics = (0 until bankWidth).map { w =>
    val compare_valid = stale_cfi_mask(w)
    val actual_taken =
      s1_update.bits.cfi_idx.valid &&
      s1_update.bits.cfi_idx.bits === w.U &&
      s1_update.bits.cfi_taken

    val old_taken = s1_update_meta.hits(w) && s1_update_meta.pred_taken(w)
    val new_taken = stale_entry.alt_taken(w)
    val direction_same = old_taken === new_taken
    val old_direction_ok = old_taken === actual_taken
    val new_direction_ok = new_taken === actual_taken
    val target_valid = compare_valid && old_taken && new_taken && actual_taken
    val old_target = s1_update_meta.pred_target(w)
    val new_target = stale_entry.alt_target(w)
    val target_same = old_target === new_target
    val old_target_ok = target_valid && (old_target === s1_update.bits.target)
    val new_target_ok = target_valid && (new_target === s1_update.bits.target)
    val old_prediction_ok =
      old_direction_ok &&
      (!actual_taken || (old_target === s1_update.bits.target))
    val new_prediction_ok =
      new_direction_ok &&
      (!actual_taken || (new_target === s1_update.bits.target))

       Seq(
      // 第0组：方向不同旧对、方向不同新对。
      (
        compare_valid && !direction_same && old_direction_ok,
        compare_valid && !direction_same && new_direction_ok,
        false.B
      ),
      // 第1组：方向都预测为taken且实际为taken,目标不同旧对、目标不同新对、目标不同且都错。
      (
        target_valid && !target_same && old_target_ok,
        target_valid && !target_same && new_target_ok,
        target_valid && !target_same && !old_target_ok && !new_target_ok
      ),
      // 第2组：完整预测新对旧错、完整预测旧对新错。
      (
        compare_valid && !old_prediction_ok && new_prediction_ok,
        compare_valid && old_prediction_ok && !new_prediction_ok,
        false.B
      )
    )
  }
  val stale_direction_diff_old_correct_count =
    PopCount(VecInit(stale_prediction_statistics.map(slot => slot(0)._1)))
  val stale_direction_diff_new_correct_count =
    PopCount(VecInit(stale_prediction_statistics.map(slot => slot(0)._2)))
  val stale_target_diff_old_correct_count =
    PopCount(VecInit(stale_prediction_statistics.map(slot => slot(1)._1)))
  val stale_target_diff_new_correct_count =
    PopCount(VecInit(stale_prediction_statistics.map(slot => slot(1)._2)))
  val stale_target_diff_both_wrong_count =
    PopCount(VecInit(stale_prediction_statistics.map(slot => slot(1)._3)))
  val stale_prediction_new_correct_old_incorrect_count =
    PopCount(VecInit(stale_prediction_statistics.map(slot => slot(2)._1)))
  val stale_prediction_old_correct_new_incorrect_count =
    PopCount(VecInit(stale_prediction_statistics.map(slot => slot(2)._2)))

  val branch_hit_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(1)._1)))
  val branch_miss_not_taken_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(1)._2)))
  val branch_miss_taken_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(1)._3)))

  val branch_direction_correct_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(2)._1)))
  val branch_direction_incorrect_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(2)._2)))

  val branch_target_correct_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(3)._1)))
  val branch_target_incorrect_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(3)._2)))

  val jal_hit_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(4)._1)))
  val jal_miss_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(4)._2)))
  val jal_target_correct_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(4)._3)))
  val jal_target_incorrect_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(4)._4)))

  val jalr_hit_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(5)._1)))
  val jalr_miss_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(5)._2)))
  val jalr_target_correct_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(5)._3)))
  val jalr_target_incorrect_count =
    PopCount(VecInit(slot_statistics.map(slot => slot(5)._4)))

  // 累计计数器。
  // 只输出当前周期的事件数量，不在 FAuBTB 内累计。
  val faubtb_event_deltas = Seq(
    actual_cfi_count,                                  // 64
    covered_cfi_count,                                 // 65
    mispredicted_cfi_count,                            // 66
    duplicate_cfi_count,                               // 67
    branch_hit_count,                                 // 68
    branch_miss_not_taken_count,                       // 69
    branch_miss_taken_count,                           // 70
    branch_direction_correct_count,                   // 71
    branch_direction_incorrect_count,                 // 72
    branch_target_correct_count,                      // 73
    branch_target_incorrect_count,                    // 74
    jal_hit_count,                                    // 75
    jal_miss_count,                                   // 76
    jal_target_correct_count,                         // 77
    jal_target_incorrect_count,                       // 78
    jalr_hit_count,                                   // 79
    jalr_miss_count,                                  // 80
    jalr_target_correct_count,                        // 81
    jalr_target_incorrect_count,                      // 82
    stale_cfi_count,                                  // 83
    stale_direction_diff_old_correct_count,            // 84
    stale_direction_diff_new_correct_count,            // 85
    stale_target_diff_old_correct_count,               // 86
    stale_target_diff_new_correct_count,               // 87
    stale_target_diff_both_wrong_count,                // 88
    stale_prediction_new_correct_old_incorrect_count,  // 89
    stale_prediction_old_correct_new_incorrect_count   // 90
  )

  require(faubtb_event_deltas.size == FAuBTBPerfEvents.Count)

  val valid_commit_update =
    s1_update.valid && s1_update.bits.is_commit_update

  for (i <- 0 until FAuBTBPerfEvents.Count) {
    io.faubtb_events(i) :=
      Mux(valid_commit_update, faubtb_event_deltas(i), 0.U)
  }
}
