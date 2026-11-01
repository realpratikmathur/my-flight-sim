module MyFlightSim {

  # ----------------------------------------------------------------------
  # Symbolic constants for port numbers
  # ----------------------------------------------------------------------

  enum Ports_RateGroups {
    rateGroup_200Hz
    rateGroup_10Hz
    rateGroup_1Hz
  }

  deployment topology QuadFsw {

  # ----------------------------------------------------------------------
  # Subtopology imports
  # ----------------------------------------------------------------------
    import CdhCore.Subtopology
    import ComCcsds.Subtopology
    import DataProducts.Subtopology
    import FileHandling.Subtopology
    
  # ----------------------------------------------------------------------
  # Instances used in the topology
  # ----------------------------------------------------------------------
    instance chronoTime
    instance rateGroup_200Hz
    instance rateGroup_10Hz
    instance rateGroup_1Hz
    instance rateGroupDriver
    instance systemResources
    instance comDriver
    instance cmdSeq
    instance simUdp
    instance simBufferManager
    instance simLink

  # ----------------------------------------------------------------------
  # Pattern graph specifiers
  # ----------------------------------------------------------------------

    command connections instance CdhCore.cmdDisp
    event connections instance CdhCore.events
    telemetry connections instance CdhCore.tlmSend
    text event connections instance CdhCore.textLogger
    health connections instance CdhCore.$health
    param connections instance FileHandling.prmDb
    time connections instance chronoTime

  # ----------------------------------------------------------------------
  # Telemetry packets (only used when TlmPacketizer is used)
  # ----------------------------------------------------------------------

    # include "QuadFswPackets.fppi"

  # ----------------------------------------------------------------------
  # Direct graph specifiers
  # ----------------------------------------------------------------------

    connections ComCcsds_CdhCore {
      # Core events and telemetry to communication queue
      CdhCore.events.PktSend -> ComCcsds.comQueue.comPacketQueueIn[ComCcsds.Ports_ComPacketQueue.EVENTS]
      CdhCore.tlmSend.PktSend -> ComCcsds.comQueue.comPacketQueueIn[ComCcsds.Ports_ComPacketQueue.TELEMETRY]

      # Router to Command Dispatcher
      ComCcsds.fprimeRouter.commandOut -> CdhCore.cmdDisp.seqCmdBuff
      CdhCore.cmdDisp.seqCmdStatus -> ComCcsds.fprimeRouter.cmdResponseIn
      
    }

    connections ComCcsds_FileHandling {
      # File Downlink to Communication Queue
      FileHandling.fileDownlink.bufferSendOut -> ComCcsds.comQueue.bufferQueueIn[ComCcsds.Ports_ComBufferQueue.FILE]
      ComCcsds.comQueue.bufferReturnOut[ComCcsds.Ports_ComBufferQueue.FILE] -> FileHandling.fileDownlink.bufferReturn

      # Router to File Uplink
      ComCcsds.fprimeRouter.fileOut -> FileHandling.fileUplink.bufferSendIn
      FileHandling.fileUplink.bufferSendOut -> ComCcsds.fprimeRouter.fileBufferReturnIn
    }

    connections Communications {
      # ComDriver buffer allocations
      comDriver.allocate      -> ComCcsds.commsBufferManager.bufferGetCallee
      comDriver.deallocate    -> ComCcsds.commsBufferManager.bufferSendIn
      
      # ComDriver <-> ComStub (Uplink)
      comDriver.$recv                     -> ComCcsds.comStub.drvReceiveIn
      ComCcsds.comStub.drvReceiveReturnOut -> comDriver.recvReturnIn
      
      # ComStub <-> ComDriver (Downlink)
      ComCcsds.comStub.drvSendOut      -> comDriver.$send
      comDriver.ready         -> ComCcsds.comStub.drvConnected
    }

    connections FileHandling_DataProducts {
      # Data Products to File Downlink
      DataProducts.dpCat.fileOut -> FileHandling.fileDownlink.SendFile
      FileHandling.fileDownlink.FileComplete -> DataProducts.dpCat.fileDone
    }

    connections RateGroups {
      # (Feature 02) The simulator is the system clock: every datagram it sends
      # becomes one 200 Hz tick. This replaces "timer.CycleOut -> ..." from the
      # generated deployment.
      simLink.cycleOut -> rateGroupDriver.CycleIn

      # 200 Hz rate group: reserved for flight control (features 03-04)
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup_200Hz] -> rateGroup_200Hz.CycleIn

      # 10 Hz rate group
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup_10Hz] -> rateGroup_10Hz.CycleIn
      rateGroup_10Hz.RateGroupMemberOut[0] -> CdhCore.tlmSend.Run
      rateGroup_10Hz.RateGroupMemberOut[1] -> FileHandling.fileDownlink.Run
      rateGroup_10Hz.RateGroupMemberOut[2] -> ComCcsds.comQueue.run
      rateGroup_10Hz.RateGroupMemberOut[3] -> ComCcsds.aggregator.timeout
      rateGroup_10Hz.RateGroupMemberOut[4] -> CdhCore.cmdDisp.run
      rateGroup_10Hz.RateGroupMemberOut[5] -> cmdSeq.schedIn

      # 1 Hz rate group
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup_1Hz] -> rateGroup_1Hz.CycleIn
      rateGroup_1Hz.RateGroupMemberOut[0] -> CdhCore.$health.Run
      rateGroup_1Hz.RateGroupMemberOut[1] -> ComCcsds.commsBufferManager.schedIn
      rateGroup_1Hz.RateGroupMemberOut[2] -> DataProducts.dpBufferManager.schedIn
      rateGroup_1Hz.RateGroupMemberOut[3] -> DataProducts.dpWriter.schedIn
      rateGroup_1Hz.RateGroupMemberOut[4] -> DataProducts.dpMgr.schedIn
      rateGroup_1Hz.RateGroupMemberOut[5] -> systemResources.run
      rateGroup_1Hz.RateGroupMemberOut[6] -> simBufferManager.schedIn
      rateGroup_1Hz.RateGroupMemberOut[7] -> simLink.schedIn
    }

    connections CdhCore_cmdSeq {
      # Command Sequencer
      cmdSeq.comCmdOut -> CdhCore.cmdDisp.seqCmdBuff
      CdhCore.cmdDisp.seqCmdStatus -> cmdSeq.cmdResponseIn
    }

    connections Simulator {
      # Udp driver buffer pool
      simUdp.allocate   -> simBufferManager.bufferGetCallee
      simUdp.deallocate -> simBufferManager.bufferSendIn

      # Simulator -> flight software: datagrams in, buffers back
      simUdp.$recv                -> simLink.drvReceiveIn
      simLink.drvReceiveReturnOut -> simUdp.recvReturnIn
      simUdp.ready                -> simLink.drvConnected

      # Flight software -> simulator (used starting in feature 03)
      simLink.drvSendOut -> simUdp.$send
    }

    connections QuadFsw {

    }

  }

}
