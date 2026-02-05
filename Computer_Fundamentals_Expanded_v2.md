# Computer Fundamentals — Highly Detailed One-Line Notes (MCQ-focused)

> Exam-focused one-liners for rapid revision and flashcard creation.  
> Covers: Computer System & Types; Components & Architecture; CPU Components (CU, ALU, Registers); Input Devices; Processor; Memory; Storage Devices; Output Devices; Peripheral Interfaces; Practical mnemonics & exam tactics.  
> NOTE: This version (v2) merges the detailed CU / ALU / Register File notes into the main Computer Fundamentals file.

---

## Additions & Verifications (new facts added after review)
- Reviewed existing content and added missing core/edge concepts often tested in competitive MCQs: boot firmware & process, kernel vs user mode & system calls, device driver basics, synchronization primitives (mutex/semaphore/spinlock), non-maskable interrupts, secure boot/UEFI, BIOS differences, instruction set examples (x86/ARM/MIPS), memory barriers/consistency basics, inode concept and filesystem permissions detail, software RAID vs hardware RAID controller differences, encryption at rest (LUKS/BitLocker), container vs VM key differences, lightweight virtualization (containers, namespaces, cgroups), Ethernet basics (MAC address, auto-negotiation, common speeds), and GPIO/serial/parallel legacy interfaces. These were integrated and expanded across relevant sections.

---

## Computer System and Types (expanded)
- Computer system = hardware + system software + application software + data + users + procedures + optional network.  
- Basic computing function = Input → Process → Output → Storage (IPO model).  
- Embedded system = dedicated computing inside a device (real-time constraints, low power, examples: router, thermostat).  
- Real-time systems: hard real-time (missed deadlines catastrophic), soft real-time (deadline miss tolerable with degraded service).  
- Microcontroller vs microprocessor: microcontroller bundles CPU+RAM+ROM+I/O on one chip; microprocessor requires external peripherals.  
- Microcomputer = PC/laptop for single-user general-purpose tasks.  
- Workstation = high-performance single-user system for engineering/graphics.  
- Server = headless machine optimized for uptime (web, DB, file services).  
- Minicomputer = historical mid-range multi-user system (now largely servers).  
- Mainframe = enterprise-grade system for high throughput & many concurrent transactions.  
- Supercomputer = extremely high-performance for FLOPS-heavy tasks (simulations, weather).  
- Analog computer = processes continuous signals; digital = discrete binary values; hybrid = both.  
- Special-purpose computer = built for a single application (e.g., ATM controller) vs general-purpose = runs varied programs.  
- Batch processing = jobs collected and processed in groups; interactive processing = immediate user response.  
- Multiprocessing: symmetric multiprocessing (SMP) = all CPUs equal; asymmetric = assigned roles.  
- Multi-threading: concurrency inside process; shared memory requires synchronization (locks, semaphores).  
- Parallel computing: data parallelism (same op on many data) vs task parallelism (different tasks concurrently).  
- SIMD vs MIMD: SIMD = single instruction multiple data (vectorized ops), MIMD = multiple independent instruction streams.  
- Cluster vs grid vs cloud: cluster = tightly-coupled nodes; grid = loosely-shared resources; cloud = elastic service delivery.  
- Thin client = minimal local processing; fat client = heavy local processing and offline capability.  
- Scale-up vs scale-out: scale-up = stronger hardware; scale-out = add more machines (cloud favors scale-out).  
- High-availability design = redundancy, failover clusters, heartbeat monitoring, quorum.  
- Load balancing methods: round-robin, least connections, IP-hash; sticky sessions require affinity.  
- Fault tolerance components: redundant power, RAID, ECC memory, NIC teaming.  
- Licensing models: proprietary, per-seat, subscription, open-source licenses (GPL, MIT, Apache).  
- Availability vs reliability: availability = uptime percentage; reliability = MTBF/MTTR relationship.  
- Common exam traps: distinguish "embedded" vs "general-purpose" and "reliability" vs "availability".  
- Virtual machines (VMs) emulate full hardware; containers share the host kernel (namespaces & cgroups) and are more lightweight.  
- Key container differences: faster startup, less overhead, kernel sharing — but weaker isolation vs VMs.  
- Software virtualization uses hypervisors (Type-1 bare-metal, Type-2 hosted); container runtimes (Docker, containerd) rely on kernel features.  
- Cloud service model distinctions refresher: IaaS = virtual machines & block storage; PaaS = managed runtime/platform; SaaS = complete applications.  

---

## Components and Architecture of Computers (expanded)
- CPU primary components = Control Unit (CU) + Arithmetic Logic Unit (ALU) + Register File.  
- Instruction cycle = Fetch → Decode → Execute → Memory/Writeback.  
- Program Counter (PC) stores address of next instruction.  
- Common registers = PC, IR (instruction register), MAR, MDR, SP, BP/frame pointer, FLAGS.  
- System bus types = Data bus, Address bus, Control bus; address bus width limits addressable memory.  
- Von Neumann architecture uses same memory for instructions & data (Von Neumann bottleneck possible).  
- Harvard architecture separates instruction & data memory for parallel fetch (common in DSPs).  
- ISA (Instruction Set Architecture) describes instruction formats, registers, addressing modes.  
- RISC = Reduced Instruction Set Computer: simple, many registers, fixed-length instructions, pipeline-friendly (e.g., ARM).  
- CISC = Complex Instruction Set Computer: complex multi-step instructions, microcode layer (e.g., x86).  
- Microarchitecture = pipeline depth, issue width, caches, execution units (implementation of ISA).  
- Pipelining overlaps stages; hazards = data, control, structural; resolved by forwarding, stalls, branch prediction.  
- Pipeline hazards: RAW (read-after-write), WAR (write-after-read), WAW (write-after-write).  
- Superscalar CPUs issue multiple instructions per cycle using multiple ALUs/execution units.  
- Out-of-order execution reorders instructions to maximize execution unit use; uses ROB and register renaming.  
- Branch prediction types: static vs dynamic (2-bit saturating counters); misprediction penalty flushes pipeline.  
- Speculative execution executes predicted branch paths; vulnerabilities: Spectre, Meltdown.  
- SIMD = vector instructions (SSE/AVX) process multiple data elements per instruction.  
- Cache hierarchy: L1 (fastest, smallest), L2, L3 (largest, slowest); larger levels reduce average access time.  
- Cache associativity: direct-mapped, N-way set associative, fully associative; higher associativity reduces conflict misses.  
- Cache write policies: write-through (immediate memory write) vs write-back (defer & mark dirty).  
- Cache replacement: LRU, FIFO, Random; replacement affects miss rate.  
- TLB (Translation Lookaside Buffer) caches virtual→physical page mappings; TLB misses cause page table walk.  
- DMA (Direct Memory Access) allows devices to transfer data without CPU involvement.  
- Interrupts are asynchronous signals to CPU to run ISR; interrupt priority and masking control handling.  
- Non-maskable interrupt (NMI) is high-priority and cannot be ignored; used for critical hardware failures.  
- Context switch = save/restore CPU state; expensive (cache/TLB flush effects).  
- Clock rate measured in Hz (GHz typical); CPI and instruction count determine CPU time.  
- CPU time = Instruction count × CPI × Clock period.  
- Amdahl’s Law: overall speedup limited by serial fraction; Speedup = 1 / (S + P/N).  
- Little’s Law for queues: L = λ × W (average number in system = arrival rate × average time in system).  
- Bus arbitration methods: centralized vs distributed; bus mastering allows DMA-capable devices to initiate transfers.  
- NUMA (Non-Uniform Memory Access) = memory access latency varies by node; OS-aware allocation improves locality.  
- Cache coherence protocols (MESI/MOESI) keep multiple caches consistent in multicore systems.  
- Power management: ACPI states (S0–S5), P-states control CPU frequency/voltage, C-states control idle sleep levels.  
- TPM (Trusted Platform Module) provides secure key storage & measured boot.  
- Virtualization hardware support: Intel VT-x, AMD-V; IOMMU for device assignment.  
- Microcode enables CPU vendor updates/fixes for errata without hardware replacement.  
- Typical exam specifics: RISC traits = simple ops & pipelining, CISC traits = complex ops & microcode.  
- Kernel mode vs user mode: kernel mode (privileged) executes OS code; user mode runs applications and uses system calls to request services.  
- System call examples: open/read/write/exec/exit in POSIX — OS switches to kernel mode to service them.  
- Device drivers: kernel modules or user-space drivers that mediate hardware access; driver types include character, block, network drivers.  
- Synchronization primitives: mutex (mutual exclusion), semaphore (counting), spinlock (busy-wait for short critical sections), barrier (synchronization point).  
- Memory barriers/fences ensure ordering of memory operations on weakly-ordered architectures (important for concurrent programs).  
- Return stack buffer (RSB) and branch target buffer (BTB) are microarchitectural predictors used for return/branch prediction.  
- Hardware prefetchers fetch memory lines based on access patterns to reduce cache miss latency.  
- Instruction set examples to remember: x86/x86-64 (Intel/AMD, CISC), ARM (RISC, mobile & servers), MIPS (RISC, academic/embedded).

---

## CPU Components — CU, ALU, Register File (detailed one-liners)
> (Merged and placed here for clear CPU-focused review)

### Control Unit (CU)
- Control Unit orchestrates execution: fetches instructions, decodes them, issues control signals to CPU units and memory.  
- CU modes: hardwired control (fast, fixed) vs microprogrammed control (microcode, flexible).  
- CU interprets opcodes and selects micro-operations (fetch operands, select ALU op, writeback).  
- CU generates timing & control signals for buses, memory, I/O and peripheral synchronization.  
- CU handles instruction sequencing: increments PC, manages jumps/branches, calls/returns.  
- CU interacts with the interrupt controller: saves PC/state, vectors to ISR, restores on return.  
- CU enforces privilege levels: traps illegal instructions from user mode to kernel mode.  
- CU implements pipeline control: issues stall/flush on hazards and sequences pipeline stages.  
- CU supports power management by issuing idle/low-power state transitions (C-states via ACPI).  
- CU is responsible for I/O instruction handling and special instructions (e.g., privileged ops).  
- CU may include branch prediction control logic (interface to branch predictor unit).  
- CU coordinates memory ordering and enforces memory barriers when required by ISA.  
- In microprogrammed designs, CU fetches microinstructions from control store (microcode ROM/RAM).  
- CU size/complexity differs: simple microcontrollers have minimal CU; complex general-purpose CPUs have large CU logic.

### Arithmetic Logic Unit (ALU)
- ALU performs arithmetic (add, subtract, multiply, divide) and logical (AND, OR, XOR, NOT) operations.  
- ALU inputs come from register file or immediate fields; outputs written back to registers or memory.  
- ALU also provides shift/rotate operations (logical/arithmetic shifts) used in multiplication/division and bit manipulation.  
- ALU signals common status flags: Zero (Z), Sign/Negative (N), Carry (C), Overflow (V/OV), Parity (optional).  
- Carry flag indicates unsigned overflow; overflow flag indicates signed overflow in two's complement arithmetic.  
- ALU can implement integer arithmetic in combinational logic; multiply/divide may be iterative or use dedicated hardware (booth multiplier, divider).  
- Floating-point operations are usually executed in a separate FPU (floating-point unit) with IEEE-754 semantics.  
- ALU latency affects instruction throughput; complex ops may require multi-cycle ALU sequences or separate execution units.  
- ALU pipelines or multiple ALUs (integer ALU, address generation unit) support superscalar execution.  
- ALU implements bitwise operations used in masks, flags, and low-level network/IO tasks.  
- ALU often supports conditional operations (set-if-less, compare-and-set) for branching and comparisons.  
- Carry-propagate adders (RCA) are simple but slower; carry-lookahead or prefix adders (Kogge-Stone) are faster for wide operands.  
- ALU design choices affect power, area, and speed trade-offs in microarchitecture.  
- ALU interfaces to condition-code or status register used by CU to make branch decisions.

### Register File (overview)
- Register file = small, very fast storage inside CPU used for operands, addresses, and intermediate results.  
- Types: general-purpose registers (GPRs), special-purpose registers (PC, SP, FP, FLAGS), control registers (CR3 in x86), vector/simd registers, floating-point registers.  
- Register width typically matches ISA word size: 32-bit registers for x86-32, 64-bit for x86-64/ARM64.  
- Register file is multi-ported to allow multiple simultaneous reads/writes per cycle (important in superscalar CPUs).  
- Register file access latency typically 1 cycle in modern CPUs; port count and banking influence timing.  
- Register naming: examples x86 RAX/RBX/RCX...; ARM X0–X30; registers have calling convention roles (args, return, temp, preserved).  
- Special registers:  
  - PC (Program Counter / Instruction Pointer) — holds address of next instruction.  
  - SP (Stack Pointer) — points to top of call stack.  
  - BP/FP (Base/Frame Pointer) — assists structured stack frame access.  
  - IR (Instruction Register) — holds fetched instruction being decoded.  
  - MAR (Memory Address Register) & MDR (Memory Data Register) — used in some microarchitectures between CPU and memory.  
  - FLAGS/Status Register — holds condition flags (Z, C, N, V) and control bits (interrupt enable).  
- Register windows (SPARC) provide fast procedure call linkage by shifting register window pointers rather than saving/restoring registers.  
- Register renaming eliminates false dependencies (WAW/WAR) by mapping architectural registers to a larger pool of physical registers.  
- Architectural vs physical registers: ISA-defined vs larger physical file used in out-of-order cores for renaming.  
- Register aliasing / renaming implemented in register alias table (RAT) to avoid pipeline stalls due to false dependencies.  
- Shadow registers / banked registers used in interrupt/exception handling to avoid context save overhead (common in real-time/embedded CPUs).  
- Call/return sequence uses SP, return address stored on stack or in link register (ARM LR).  
- Register allocation in compilers affects number of spills to memory; more registers reduce memory traffic.  
- Vector/SIMD registers (e.g., XMM/YMM/ZMM or ARM Neon) are wide registers (128/256/512-bit) used for data-parallel ops.  
- Floating-point registers often separate from integer registers and follow FPU conventions (stack-based x87 vs register-based SSE/AVX).  
- Register file hazards: read-after-write (RAW) genuine dependency; register renaming converts false dependencies into independent physical registers.  
- Context switch must save/restore user-visible registers (and sometimes floating-point/vector state) — cost depends on register count.  
- Register file size impacts wakeup/select logic complexity in out-of-order cores (more physical registers → larger scoreboard).

### Important Register-related Concepts & MCQ Facts
- PC holds next instruction address; modifying PC = branch/jump.  
- Status/flags register used for conditional branches (e.g., branch-if-zero uses Z flag).  
- MAR/MDR used in classic CPU models to interface with memory read/write cycles.  
- IR contains the fetched instruction during decode stage.  
- SP points to top of stack; push/pop adjust SP and access memory at SP.  
- Register renaming helps avoid false dependencies and improve instruction-level parallelism.  
- Number of general-purpose registers differs by ISA (e.g., x86-64: 16 general-purpose, ARM64: 31 general-purpose).  
- Shadow/banked registers speed up context changes (interrupts) by avoiding full register save/restore.  
- SIMD/vector registers store packed data; operations apply same ALU operation across lanes.  
- RISC architectures usually expose more registers to reduce memory access frequency; CISC historically exposed fewer (modern x86-64 has many regs).  
- Typical exam trick: "Is the program counter a register?" — Yes, PC is a special-purpose register.

### Short Exam Tactics (one-line)
- Memorize roles: CU = orchestrator & sequencer; ALU = arithmetic/logic executor & flags producer; Registers = fastest storage & operand source/destination.  
- Know examples of special registers (PC, SP, IR, FLAGS) and common register counts for mainstream ISAs.  
- Understand register renaming purpose — to remove false dependencies WAW/WAR and enable out-of-order execution.  
- Be able to identify which unit (CU/ALU/register file) is responsible for actions in short MCQs (e.g., "Which unit sets the Zero flag?" → ALU).  
- Practice small diagrams: instruction fetch (PC→MAR→memory→IR), decode (CU parses opcode), execute (ALU uses registers), writeback (register file updated).

---

## Input Devices (expanded)
- Input device categories = Human Interface Devices (HID), sensors, data capture, network input.  
- Keyboard: scans key matrix; ghosting & rollover issues; USB HID/PS/2 differences (PS/2 sometimes supports true N-key rollover).  
- Mouse: optical vs laser; DPI indicates sensitivity; polling rate affects responsiveness for gaming.  
- Touchscreen: resistive (pressure-based) vs capacitive (conductive touch); gestures recognized by OS.  
- Scanner: resolution in DPI; OCR converts scanned images to editable text; scan quality affects OCR accuracy.  
- Barcode scanners: 1D (UPC) vs 2D (QR codes); QR supports error correction (ECC).  
- MICR (magnetic ink) used in cheque processing; reliable for bank automation.  
- Camera sensors: CCD vs CMOS (CMOS dominates for integration & power efficiency).  
- Microphone properties: sample rate (e.g., 44.1 kHz), bit depth (16/24-bit), SNR affects capture quality.  
- ADC (Analog-to-Digital Converter) resolution = number of bits (8/10/12/16), affects quantization precision.  
- Nyquist sampling theorem: sampling rate > 2 × highest frequency to avoid aliasing.  
- Biometric devices: fingerprint, iris, face; performance measured by FAR (false accept rate) & FRR (false reject rate).  
- Smart card readers use ISO/IEC 7816; contactless uses NFC (ISO 14443).  
- Accelerometers/gyroscopes communicate via I2C/SPI in embedded systems.  
- Hot-plug vs hot-swap: hot-swap requires hardware support and safe removal without shutdown (hot-pluggable may not be hot-swappable).  
- Polling vs interrupt-driven input: polling consumes CPU cycles; interrupts are event-driven and efficient.  
- USB polling intervals vary by device class (e.g., low-speed vs full-speed).  
- Device drivers run in kernel mode for direct hardware access; user-space drivers possible via UIO frameworks.  
- Input latency measured in ms from event to system processing — critical for real-time/gaming.  
- GPIO (General Purpose I/O) lines provide simple pin-level interfacing in embedded systems; controlled by drivers and often accessed via sysfs or user-space libraries.  
- Serial ports (UART/RS-232) and legacy parallel ports still appear in some industrial exam scenarios — know basic RS-232 signal levels and use-cases.  
- Hotkey/hardware scan codes vs OS-mapped keycodes: low-level scan code differs from high-level key events in OS.

---

## Processor (expanded)
- Clock frequency (GHz) × IPC (instructions per cycle) × instruction count = performance (ignoring memory effects).  
- Core count improves throughput for parallel workloads; single-thread performance depends on IPC & frequency.  
- SMT/Hyper-Threading provides multiple logical threads per physical core but shares execution resources.  
- Turbo/Boost raises clocks temporarily under power/thermal headroom.  
- Thermal Design Power (TDP) indicates expected heat dissipation; influences cooling design.  
- SIMD vector units (SSE/AVX) improve performance on data-parallel operations (multimedia, ML).  
- GPU vs CPU: GPU excels at massively parallel, data-parallel workloads; CPU for general-purpose control logic.  
- GPU memory (VRAM) bandwidth critical for high-resolution graphics and compute kernels.  
- Out-of-order execution increases throughput but requires mechanisms for in-order retirement (ROB).  
- Branch predictors (two-bit, global/local) reduce misprediction rate; misprediction flush penalty wastes cycles.  
- Speculative execution vulnerabilities (Spectre/Meltdown) arise from side channels exposing privileged data.  
- Secure enclaves (Intel SGX, ARM TrustZone) isolate sensitive code/data from OS.  
- Hardware RNG (e.g., RDRAND) offers fast random numbers; seed quality matters for cryptography.  
- Micro-op fusion in complex ISAs reduces decode/dispatch overhead for paired instructions.  
- Die shrink (nm process) increases transistor density, reduces power per transistor, allows higher clocks or more cores.  
- Instruction length fixed (RISC) vs variable (CISC) influences decode complexity.  
- Hardware virtualization support reduces VM exit/entry overhead compared to pure software virtualization.

---

## Memory (expanded)
- Register access = ~1 cycle; L1 cache ~1–5 cycles; L2 ~5–20 cycles; main RAM tens–hundreds cycles (numbers vary by architecture).  
- DRAM types: DDR3, DDR4, DDR5 — higher generations provide higher throughput and density.  
- SRAM used for caches; faster but more expensive than DRAM.  
- Page size commonly 4 KiB; hugepages (2 MiB/1 GiB) reduce TLB pressure for large working sets.  
- TLB caches virtual→physical translations; TLB misses trigger page table walk (expensive).  
- Page fault occurs when virtual page not mapped; OS must load page from disk (swap) — very slow.  
- Swap usage leads to thrashing if working set > RAM causing heavy disk I/O and low throughput.  
- ECC memory detects and corrects single-bit errors (SECDED), used in servers to prevent silent data corruption.  
- Memory channels (dual/quad) increase bandwidth; matching DIMMs recommended for interleaving.  
- Memory timings e.g., CAS Latency (CL) impact latency; lower CL generally better per clock cycle.  
- Memory leak = allocated memory not freed; eventually causes OOM and may crash process/system.  
- Heap vs stack: stack for function frames (LIFO), limited size; heap for dynamic allocations, requires management/garbage collection.  
- Cache associativity & line size affect spatial/temporal locality behavior; larger lines help sequential access but can increase miss penalty for random access.  
- Write amplification in SSDs increases actual writes vs host writes; wear-leveling counteracts uneven write distribution.  
- Non-volatile memory types: NVDIMM, persistent memory blur RAM vs storage boundary for fast persistence.  
- Memory consistency models: strong (sequential consistency) vs weak (relaxed) — atomic operations & fences required to ensure order on weak models.  
- Memory barriers (mfence, lfence in x86) enforce ordering for concurrent code on weakly-ordered systems.

---

## Storage Devices (expanded)
- HDD = spinning platters + read/write head; key metrics: seek time, rotational latency, transfer rate, throughput.  
- Average rotational latency = 0.5 × rotation_period (e.g., 7200 RPM → rotation period 8.33 ms → avg latency ~4.17 ms).  
- SSD uses NAND flash; faster random I/O, limited write endurance (TBW/DWPD) based on cell type (SLC/MLC/TLC/QLC).  
- NAND cell types: SLC (1 bit/cell, highest endurance), MLC (2 bits), TLC (3 bits), QLC (4 bits, lower endurance).  
- NVMe uses PCIe lanes for high throughput & low latency; NVMe supports many submission/completion queues for parallelism.  
- SATA III ≈ 6 Gbit/s (~600 MB/s theoretical); PCIe Gen3 x4 roughly 3.9 GB/s, Gen4 x4 ~7.9 GB/s (theoretical).  
- TRIM command informs SSD which blocks no longer in use; helps maintain write performance.  
- SSD controllers perform wear-leveling, bad-block remapping, and garbage collection; these affect latency.  
- SMART attributes in drives report health (reallocated sector count, wear_leveling_count, power_on_hours).  
- RAID 0 = striping (no redundancy), RAID 1 = mirroring (one disk failure tolerance), RAID 5 = distributed parity (single disk tolerance), RAID 6 = double parity (two disk tolerance), RAID 10 = mirrors then stripes (performance + redundancy).  
- Hardware RAID uses dedicated controller & battery-backed cache; software RAID (mdadm/ZFS) uses CPU but is flexible — exam may ask differences.  
- Filesystem examples: FAT32 (4 GB file limit), NTFS (Windows journaling + ACLs), ext4 (Linux journaling), XFS (scalable), Btrfs/ZFS (modern features: checksums, snapshots).  
- Inode concept: filesystem metadata structure (permissions, owner, timestamps, pointers to data blocks); inode table size limits number of files.  
- Filesystem permissions: Unix rwx for owner/group/others; chmod numeric (e.g., 755, 644) changes permissions.  
- Mount options: read-only (ro), noexec, nosuid, nodev — used to constrain behavior and improve security.  
- Encryption at rest: LUKS (Linux Unified Key Setup) commonly used on Linux; BitLocker on Windows — protects data if disks stolen.  
- Partitioning schemes: MBR (legacy, 4 primary partitions, ≤2 TB), GPT (modern, large disk support, many partitions).  
- LVM provides logical volumes for flexible resizing, snapshots, striping, mirroring on Linux.  
- Snapshot types: copy-on-write (COW) vs redirect-on-write; differs in snapshot performance & storage use.  
- Backup types: full (complete), incremental (changes since last backup), differential (changes since last full backup).  
- Backup strategies & RTO/RPO: RTO = Recovery Time Objective; RPO = Recovery Point Objective.  
- Replication: synchronous (strong consistency, higher latency) vs asynchronous (eventual consistency, lower latency).  
- Erasure coding (Reed-Solomon) used in distributed storage for space-efficient redundancy vs RAID local solutions.  
- Object storage (S3) is key-value with metadata and eventual consistency; block storage is for low-level DB storage.  
- Network storage protocols: NFS (file-level UNIX), SMB/CIFS (Windows shares), iSCSI (block over IP), Fibre Channel (SAN).  
- Storage performance metrics: throughput (MB/s), IOPS (IO operations per second), latency (ms) — tune based on workload type (sequential vs random).  
- Encryption and secure erase considerations: secure erase commands vs cryptographic erase (delete keys to render data unreadable).  
- SMART and periodic health checks together with backups prevent data-loss surprises; don't rely only on RAID for backup.

---

## Output Devices (expanded)
- Display types: CRT (legacy), LCD, LED-backlit LCD, OLED; differences in contrast, response time, power.  
- Resolution examples: 1920×1080 (Full HD), 2560×1440 (QHD), 3840×2160 (4K UHD).  
- Refresh rate (Hz) vs frame rate (fps); high refresh (120/144/240Hz) benefits gaming.  
- Color depth: 8-bit per channel = 24-bit color (~16.7M colors); 10-bit per channel used for HDR & professional imaging.  
- Color spaces: sRGB (web), Adobe RGB (wider gamut for print), YUV (video).  
- Display interfaces: VGA (analog), DVI (digital), HDMI (audio + video), DisplayPort (high bandwidth), USB-C Alt Mode (video + data).  
- GPU pipeline: vertex processing → rasterization → fragment shading → output merger; programmable shaders handle modern effects.  
- Framebuffer, double buffering, vsync avoid tearing; adaptive sync (G-Sync/FreeSync) reduces stutter/tearing.  
- Printer types: inkjet (color photos), laser (fast text), dot-matrix (impact), thermal (receipts).  
- Printer resolution measured in dpi; halftoning approximates continuous tones via dots.  
- Audio output: CD quality = 44.1 kHz/16-bit; pro audio uses 48kHz/96kHz with 24-bit depth.  
- Output latency important in audio/video production and live systems; buffer size trade-off between latency & underrun risk.  
- Haptic output: actuators/vibration motors controlled by PWM for intensity control.  
- Accessibility outputs: screen readers, Braille displays, closed captions; critical in exam contexts for inclusive design.  
- Print protocols & drivers: PPD (PostScript Printer Description) and CUPS for Unix-like printing stacks; printer queues & spoolers manage jobs.

---

## Peripheral Interfaces & Protocols (additional)
- USB versions: USB 2.0 = 480 Mbps; USB 3.0/3.1 Gen1 = 5 Gbps; Gen2 = 10 Gbps; USB-C = connector standard.  
- PCIe lanes: x1/x4/x8/x16; bandwidth scales with lane count & PCIe generation.  
- I2C: two-wire serial bus (SDA, SCL) for low-speed sensor/peripheral comms; multi-master capable.  
- SPI: full-duplex serial interface (MOSI, MISO, SCLK, SS), faster than I2C but more pins per device.  
- UART/RS-232: asynchronous serial, common for console debug on routers/embedded devices.  
- CAN bus: robust multi-master fieldbus for automotive & industrial applications.  
- SATA: interface for HDD/SSD; AHCI supports NCQ and hot-plug; SATA III ≈ 6 Gbit/s.  
- NVMe: optimized for PCIe with many queues & low-latency commands for SSDs.  
- Thunderbolt 3 uses USB-C connector and supports 40 Gbps (PCIe + DisplayPort multiplex).  
- SMBus is a subset of I2C for system management functions.  
- Ethernet basics: MAC address (48-bit), auto-negotiation for speed/duplex, common speeds 10/100/1000/10GbE; switches forward based on MAC; hubs are collision domains.  
- Wireless peripheral protocols: Bluetooth versions (2.0 legacy, 4.0 BLE, 5.0 increased range & throughput) and Wi-Fi standards (802.11 a/b/g/n/ac/ax).  
- GPIO, PWM and ADC pins common on microcontrollers for direct hardware control; interfaced via sysfs, devfs, or user libraries.  
- Serial console access useful for headless servers (tty, console baud rates like 9600/115200).  
- Hot-plug & hot-swap behavior depends on hardware & driver support (SATA hot-swap & USB are common examples).

---

## Practical Exam Mnemonics & One-Liners to Memorize (verify)
- ACID = Atomicity, Consistency, Isolation, Durability.  
- OSI mnemonic: Please Do Not Throw Sausage Pizza Away → Physical, Data Link, Network, Transport, Session, Presentation, Application.  
- TCP vs UDP: TCP = reliable, connection-oriented; UDP = unreliable, connectionless.  
- Common ports: HTTP 80, HTTPS 443, SSH 22, FTP 21, DNS 53, SMTP 25, MySQL 3306, RDP 3389.  
- RAID cheat-sheet: RAID 0 = speed (no redundancy); RAID 1 = mirror; RAID 5 = distributed parity (1 disk fail); RAID 6 = double parity; RAID 10 = mirror+stripe.  
- Virtual memory page size common = 4 KiB; hugepages = 2 MiB/1 GiB options.  
- SSD cell types endurance order: SLC > MLC > TLC > QLC.  
- Partitioning: MBR ≤ 2 TB + 4 primaries; GPT supports >2 TB and many partitions.  
- Instruction cycle mnemonic = Fetch, Decode, Execute, Store (FDES).  
- Cache order: Registers → L1 → L2 → L3 → RAM → SSD/HDD (memorize order, approximate latencies).  
- CPU performance formula: CPU time = (Instruction count × CPI) / Clock_frequency.

---

## Exam Tactics (verify)
- MCQ time: 45 minutes for 50 Q → ~54 seconds per question; prioritize easy ones and mark doubtful for review.  
- Negative marking = −20% for incorrect answers; eliminate wrong choices before guessing; avoid blind guesses.  
- Convert each one-liner into a flashcard (term → one-line answer).  
- Memorize numeric defaults (ports, page sizes, standard speeds) and key law/policy names where applicable.  
- Practice sets under timed conditions to build speed and accuracy.

---

## Closing / Next steps
- I merged the CU / ALU / Register File notes into the Computer Fundamentals expanded file (v2) and verified/added missing CPU- and architecture-related items.  
- Next I can:  
  - Convert this Markdown into a Google Doc or downloadable .docx,  
  - Generate flashcards (CSV/Anki) from every one-liner, or  
  - Produce 150–200 MCQs with answers & short explanations for this chapter.