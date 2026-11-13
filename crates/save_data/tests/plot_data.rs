use mchprs_save_data::plot_data::{
    ChunkData, ChunkSectionData, PlotData, PlotLoadError, Tps, WorldSendRate,
};
use std::borrow::Cow;
use std::fs;

#[test]
fn load_rejects_unknown_block_state() {
    let unknown = 30_000;
    let mut data = vec![0; 1024];
    data[0] = u64::from(unknown);
    let section = ChunkSectionData {
        data: Cow::Owned(data),
        palette: Cow::Owned(Vec::new()),
        bits_per_block: 15,
        block_count: 1,
    };
    let plot = PlotData {
        tps: Tps::Unlimited,
        world_send_rate: WorldSendRate::default(),
        chunk_data: vec![ChunkData {
            sections: vec![Some(section)],
            block_entities: Cow::default(),
        }],
        pending_ticks: Vec::new(),
    };
    let path = std::env::temp_dir().join(format!("mchprs-plot-{}", std::process::id()));
    plot.save_to_file(&path).unwrap();
    let loaded = PlotData::load_from_file(&path);
    fs::remove_file(&path).unwrap();
    assert!(matches!(loaded, Err(PlotLoadError::UnknownBlockState(id)) if id == unknown));
}
