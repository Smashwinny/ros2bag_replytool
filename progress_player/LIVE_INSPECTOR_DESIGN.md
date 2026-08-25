# ESKF live inspector design

The inspector is replay-only tooling. It incrementally tails the current
`eskf_process_log/v2`, summarizes state and computation matrices, and never publishes an
algorithm input or mutates ESKF state.

Current-file size is read once per UI refresh. Total `runs/` usage is scanned in a worker thread so
large histories cannot block Tk or ROS polling. Cleanup requires confirmation, operates only on direct
children of the explicit `--runs-dir`, and preserves both the caller's run and every directory carrying
an `.active` marker. The replay wrapper creates the marker before children start and removes it after
they stop.

Opening the directory and deleting completed logs are user-triggered desktop actions. Deletion is
permanent. Unit tests cover partial JSON lines, truncation, summaries, byte formatting, path rejection,
and preservation of simultaneous Domain runs.
