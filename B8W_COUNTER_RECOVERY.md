# b8w neutral rate-rejection recovery

The b8v route `00000000--5142a6c194` recorded 197 rejected private-command
bursts. Diagnostic receipts proved that 191 bursts began with a valid command
arriving 12.052–14.992 ms after the prior accepted command, below the Panda's
15 ms rate floor. The rejected counter was not recorded, so 813 later commands
were rejected as counter mismatches until the 100 ms reset window elapsed.

b8w keeps the 15 ms accepted-frame floor. An otherwise valid brake frame that
arrives too early is still rejected, is never staged, and is never placed on a
physical CAN queue. If its counter is exactly the next valid input counter, the
Panda records only that observed counter. The timestamp of the last accepted
cycle is not advanced.

This means:

- accepted private cycles remain at least 15 ms apart;
- a rapid but correctly ordered input stream cannot increase the accepted rate;
- a skipped or out-of-order early counter cannot use the recovery path;
- invalid source state, checksum, payload, or command frames cannot use it;
- the dashboard and torque frames of a rejected cycle remain blocked;
- Jeep longitudinal actuation remains hard-coded off.

Focused tests cover one early rejection, repeated rapid attempts, rejected
dashboard/torque stages, and an out-of-order early counter. Default and ELM
cross-policy tests verify that the private messages remain unavailable outside
the explicit Chrysler shadow safety parameter.

