use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Stage {
    LowerCircuit,
    WriteVerilatorCircuit,
    GenerateVerilatorModel,
    CompileVerilatorModel,
    StartVerilatorCircuit,
    GenerateVerilog,
    OpenBoardConnection,
    SynthesizeLogic,
    WaitForDeviceDatabase,
    BuildDeviceDatabase,
    AssembleDeviceDatabase,
    PlaceAndRoute,
    GenerateConfigurationFrames,
    WriteBitstream,
    ProgramBoard,
    VerifyCircuit,
}

impl Stage {
    pub(crate) fn name(self) -> &'static str {
        match self {
            Stage::LowerCircuit => "Lowering circuit",
            Stage::WriteVerilatorCircuit => "Writing Verilator circuit",
            Stage::GenerateVerilatorModel => "Generating Verilator model",
            Stage::CompileVerilatorModel => "Compiling Verilator model",
            Stage::StartVerilatorCircuit => "Starting Verilator circuit",
            Stage::GenerateVerilog => "Generating Verilog",
            Stage::OpenBoardConnection => "Opening board connection",
            Stage::SynthesizeLogic => "Synthesizing logic",
            Stage::WaitForDeviceDatabase => "Waiting for device database",
            Stage::BuildDeviceDatabase => "Building device database",
            Stage::AssembleDeviceDatabase => "Assembling device database",
            Stage::PlaceAndRoute => "Placing and routing",
            Stage::GenerateConfigurationFrames => "Generating configuration frames",
            Stage::WriteBitstream => "Writing bitstream",
            Stage::ProgramBoard => "Programming board",
            Stage::VerifyCircuit => "Verifying circuit",
        }
    }
}

impl fmt::Display for Stage {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.name())
    }
}
