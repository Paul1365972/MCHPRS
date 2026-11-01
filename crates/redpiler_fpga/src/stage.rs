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
            Stage::WriteVerilatorCircuit => "Writing circuit",
            Stage::GenerateVerilatorModel => "Verilating",
            Stage::CompileVerilatorModel => "Compiling model",
            Stage::StartVerilatorCircuit => "Starting model",
            Stage::GenerateVerilog => "Writing Verilog",
            Stage::OpenBoardConnection => "Opening board",
            Stage::SynthesizeLogic => "Synthesizing",
            Stage::WaitForDeviceDatabase => "Waiting for database",
            Stage::BuildDeviceDatabase => "Building database",
            Stage::AssembleDeviceDatabase => "Assembling database",
            Stage::PlaceAndRoute => "Placing and routing",
            Stage::GenerateConfigurationFrames => "Generating frames",
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
