`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sonar_bus48_pack_sys
//------------------------------------------------------------------------------
// ROLE
//   SYS-domain 48-bit sonar telemetry snapshot packer.
//
// PURPOSE
//   Publish a compact event-driven telemetry snapshot used by downstream
//   display, painter, and radar consumers.
//
// FIELD MAP
//   [47:32] dist_mm
//   [31:22] age_ms
//   [21:12] period_ms
//   [11:6]  drop_count
//   [5:0]   no_target_count
//
// PUBLICATION POLICY
//   - sonar_bus48 is a held snapshot register.
//   - sonar_bus48_upd is a one-cycle publication pulse.
//   - A publication occurs on:
//       1) sample_vld     : new accepted filtered sonar sample
//       2) no_target_evt  : explicit no-target event
//
// WHY THIS POLICY MATTERS
//   Downstream SYS->VID snapshot CDC expects a stable payload plus a clean
//   update pulse. Therefore this module does not continuously rewrite the bus
//   every cycle. Instead, it captures a coherent telemetry snapshot only on
//   publication events and holds that value until the next publication.
//
// TIMING SEMANTICS
//   sample_vld
//     One-cycle pulse indicating a newly accepted filtered sample.
//
//   no_target_evt
//     One-cycle pulse indicating a no-target event.
//
//   sonar_bus48_upd
//     One-cycle pulse aligned to the cycle in which sonar_bus48 is updated.
//
// COUNTER POLICY
//   period_ticks
//     Captures the elapsed ticks since the previous accepted sample.
//
//   gap_ticks
//     Free-running gap counter since the last accepted sample.
//
//   drop_count
//     Saturating count of stale intervals observed since reset.
//     Incremented once on each stale-entry transition.
//
//   no_target_count
//     Saturating count of no-target events since reset.
//
// NOTES
//   - dist_mm conversion is rounded nearest using +5 before /10.
//   - age_ticks is expected to be a millisecond-domain watchdog age.
//   - age_ms and period_ms are saturated to 10 bits.
//   - drop_count and no_target_count are saturated to 6 bits.
//==============================================================================

module sonar_bus48_pack_sys #(
    parameter integer CLK_HZ = 100_000_000
)(
    input  wire        clk_sys,
    input  wire        rst_sys,
    input  wire [9:0]  filt_in_inch,
    input  wire        sample_vld,
    input  wire        stale,
    input  wire [15:0] age_ticks,
    input  wire        no_target_evt,

    output reg  [47:0] sonar_bus48,
    output reg         sonar_bus48_upd
);

    //--------------------------------------------------------------------------
    // Internal state
    //--------------------------------------------------------------------------
    reg [31:0] update_ctr;
    reg [9:0]  period_ms_last;
    reg [9:0]  age_ms_published;

    reg [5:0]  drop_count;
    reg [5:0]  no_target_count;

    reg        stale_d1;

    //--------------------------------------------------------------------------
    // Unit conversion helpers
    //--------------------------------------------------------------------------
    wire [15:0] dist_mm_u16_raw;

    // inch -> mm using 25.4 mm/in, rounded nearest
    assign dist_mm_u16_raw   = ((filt_in_inch * 16'd254) + 16'd5) / 16'd10;

    wire [9:0] age_ms_u10 =
        (age_ticks > 16'd1023) ? 10'h3FF : age_ticks[9:0];

    wire [9:0] sample_period_ms_u10 =
        (update_ctr == 32'd0) ? 10'd0 : age_ms_u10;

    //--------------------------------------------------------------------------
    // Publication-event decode
    //--------------------------------------------------------------------------
    wire stale_rise      = stale & ~stale_d1;
    wire publish_sample      = sample_vld;
    wire publish_nt          = no_target_evt;
    wire publish_age_refresh = (age_ms_u10 != age_ms_published) &&
                               (stale_rise || (age_ms_u10[3:0] == 4'd0));
    wire publish_evt         = publish_sample | publish_nt | publish_age_refresh;

    //--------------------------------------------------------------------------
    // Sequential logic
    //--------------------------------------------------------------------------
    always @(posedge clk_sys) begin
        if (rst_sys) begin
            update_ctr       <= 32'd0;
            period_ms_last   <= 10'd0;
            age_ms_published <= 10'd0;

            drop_count       <= 6'd0;
            no_target_count  <= 6'd0;

            stale_d1         <= 1'b0;

            sonar_bus48      <= 48'd0;
            sonar_bus48_upd  <= 1'b0;
        end else begin
            // Default: publication pulse is edge-like
            sonar_bus48_upd <= 1'b0;

            // Track stale edge for drop counting
            stale_d1 <= stale;

            // Count stale-entry events as drops
            if (stale_rise && (drop_count != 6'h3F))
                drop_count <= drop_count + 6'd1;

            // Count explicit no-target events
            if (no_target_evt && (no_target_count != 6'h3F))
                no_target_count <= no_target_count + 6'd1;

            // Accepted sample updates period/gap bookkeeping
            if (sample_vld) begin
                update_ctr     <= update_ctr + 32'd1;
                period_ms_last <= sample_period_ms_u10;
            end

            // Publish a coherent held snapshot only on publication events
            //
            // Field semantics at publication time:
            //   dist_mm        = current filtered distance converted to mm
            //   age_ms         = current watchdog age in milliseconds
            //   period_ms      = accepted-sample spacing in milliseconds
            //   drop_count     = saturating stale-entry count
            //   no_target_count= saturating no-target count
            //
            // Notes on same-cycle ordering:
            //   - On a sample_vld cycle, age_ticks still holds the elapsed age
            //     since the previous accepted sample, because the watchdog clears
            //     its age register on the same edge. That value is therefore the
            //     just-completed inter-sample period.
            if (publish_evt) begin
                sonar_bus48[47:32] <= dist_mm_u16_raw;
                sonar_bus48[31:22] <= age_ms_u10;
                sonar_bus48[21:12] <= publish_sample ? sample_period_ms_u10
                                                     : period_ms_last;
                sonar_bus48[11:6]  <= drop_count;
                sonar_bus48[5:0]   <= no_target_count;

                age_ms_published   <= age_ms_u10;
                sonar_bus48_upd    <= 1'b1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Unused-state sink
    //--------------------------------------------------------------------------
    // update_ctr is retained for observability/possible later export even
    // though it is not yet packed into the 48-bit bus.
    wire _unused_update_ctr;
    assign _unused_update_ctr = update_ctr[0];

endmodule

`default_nettype wire
