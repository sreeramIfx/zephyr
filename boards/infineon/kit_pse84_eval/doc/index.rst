.. zephyr:board:: kit_pse84_eval

Overview
********
The PSOC™ Edge E84 Evaluation Kit (KIT_PSE84_EVAL) enables applications to use the PSOC™ Edge E84 Series
Microcontroller (MCU) together with multiple on-board multimedia, Machine Learning (ML),
and connectivity features including custom MIPI-DSI displays, audio interfaces,
and AIROC™ Wi-Fi and Bluetooth® combo-based connectivity modules.

The PSOC™ Edge E84 MCUs are based on high-performance Arm® Cortex®-M55 including Helium DSP support,
an Ethos™-U55 NPU, and a low-power Arm® Cortex®-M33 paired with Infineon's ultra-low power NNLite
hardware accelerator. They integrate 2.5D graphics accelerators and display interfaces, while
featuring always-on acoustic activity and wake-word detection, efficient HMI operations, and
extended battery life.

The evaluation kit carries a PSOC™ Edge E84 MCU on a SODIMM-based detachable SOM board connected to
the baseboard. The MCU SOM also has 128 MB of QSP| Flash, 1GB of Octal Flash, 128MB of Octal RAM,
PSOC™ 4000T as CAPSENSE™ co-processor, and onboard AIROC™ Wi-Fi and Bluetooth® combo.

Hardware
********
For more information about the PSOC™ Edge E84 MCUs and the PSOC™ Edge E84 Evaluation Kit:

- `PSOC™ Edge E84 Arm® Cortex® Multicore SoC Website`_
- `PSOC™ Edge E84 Evaluation Kit Website`_

Kit Features:
=============

- Cortex®-M55 CPU with Helium™ DSP
- Advanced ML with Arm Ethos™-U55 NPU
- Low-Power Cortex®-M33
- NNLite ultra-low power NPU
- Analog and Digital Microphones
- State-of-the-Art Secured Enclave
- Integrated Programmer/Debugger

Kit Contents:
=============

- PSOC™ Edge E84 base board
- PSOC™ Edge E84 SOM module
- 4.3in capacitive touch display and USB camera module
- USB Type C to Type-C cable
- Two proximity sensor wires
- Four stand-offs for Raspberry Pi compatible display
- Quick start guide

Supported Features
==================

.. zephyr:board-supported-hw::

Connections and IOs
===================

Please refer to `kit_pse84_eval User Manual Website`_ for more details.

Programming and Debugging
*************************

.. NOTE::
   The ``SW6`` (``BOOT SW``) DIP switch controls the boot mode:

   - **ON** (default): Extended boot initializes the external QSPI flash and
     jumps directly to the XIP code partition. Use this position for
     **non-MCUBoot** application development.
   - **OFF**: Extended boot jumps to the MCUBoot bootloader in RRAM
     (``0x22011000``), which then initializes the external flash and validates
     the application image before jumping to it. Use this position when
     **MCUBoot is enabled**.

   On some boards this switch may be under the attached LCD screen.

.. zephyr:board-supported-runners::

The KIT-PSE84-EVAL includes an onboard programmer/debugger (`KitProg3`_) to provide debugging,
flash programming, and serial communication over USB. Flash and debug commands use OpenOCD and
require a custom Infineon OpenOCD version, that supports KitProg3, to be installed.

Please refer to the `ModusToolbox™ software installation guide`_ to install Infineon OpenOCD.

Flashing
========

Applications for the ``kit_pse84_eval/pse846gps2dbzc4a/m33`` board target can be
built, flashed, and debugged in the usual way. See
:ref:`build_an_application` and :ref:`application_run` for more details on
building and running.

Applications for the ``kit_pse84_eval/pse846gps2dbzc4a/m55``
board target need to be built using sysbuild to include the required application for the other core.

Enter the following command to compile ``hello_world`` for the CM55 core:

.. zephyr-app-commands::
   :app: samples/hello_world
   :board: kit_pse84_eval/pse846gps2dbzc4a/m55
   :goals: build flash
   :west-args: --sysbuild
   :gen-args: -DOPENOCD=path/to/infineon/openocd/bin/openocd


Debugging
=========
The path to the installed Infineon OpenOCD executable must be available to the ``west`` tool
commands. There are multiple ways of doing this. The example below uses a permanent CMake argument
to set the CMake variable ``OPENOCD``.

   .. tabs::
      .. group-tab:: Windows

         .. code-block:: shell

            # Run west config once to set permanent CMake argument
            west config build.cmake-args -- -DOPENOCD=path/to/infineon/openocd/bin/openocd.exe

      .. group-tab:: Linux

         .. code-block:: shell

            # Run west config once to set permanent CMake argument
            west config build.cmake-args -- -DOPENOCD=path/to/infineon/openocd/bin/openocd

.. zephyr-app-commands::
   :app: samples/basic/blinky
   :board: kit_pse84_eval/pse846gps2dbzc4a/m33
   :goals: debug

Once the gdb console starts after executing the west debug command, you may now set breakpoints and
perform other standard GDB debugging on the PSOC E84 CM33 core.

Secure Boot
***********

The PSOC™ Edge E84 MCU includes an extended boot stage in ROM that, on reset, jumps to the first
application image. The destination is selected by the on-board ``BOOT SW``:

- ``BOOT SW`` **OFF**: the ROM extended boot jumps to the first application located in internal
  RRAM.
- ``BOOT SW`` **ON**: the ROM extended boot jumps to the first application located in external
  flash.

In both cases the first application image must be in MCUboot image format, i.e. it must be
preceded by an MCUboot image header (magic number, header size, vector table address, image size)
and followed by the trailer with the hash/signature TLVs. Out of the box, the device is **not**
provisioned for secure boot, so the ROM extended boot only checks the image format and hash; no
cryptographic signature verification is performed against a provisioned key.

The MCUboot image format is produced automatically by the
:file:`soc/infineon/edge/pse84/pse84_metadata.cmake` helper
``pse84_add_metadata_secure_hex()``, which invokes ``imgtool sign`` with the header address,
header size and slot size derived from the devicetree memory map. By default this helper does not
pass a signing key, which is sufficient for a non-provisioned device.

Enabling Secure Boot
====================

To enable real signature verification by the ROM extended boot, the device must be reprovisioned.
Follow sections **2.2.1**, **2.2.2** and **2.2.3** of the
`PSOC™ Edge Security Getting Started Application Note`_ to:

#. Generate (or import) the OEM signing key pair.
#. Provision the device with the corresponding public key and lifecycle transition.
#. Program the desired security counter / anti-rollback value.

After the device has been reprovisioned, the
``pse84_add_metadata_secure_hex()`` function in
:file:`soc/infineon/edge/pse84/pse84_metadata.cmake` must be updated so that ``imgtool sign``
also receives the signing key and a security counter. The relevant additions are:

.. code-block:: none

   ${PYTHON_EXECUTABLE} ${IMGTOOL} sign --version "0.0.0+0"
     --header-size ${header_size} --erased-val 0xff --pad-header
     --slot-size ${slot_size} --hex-addr ${header_addr}
     --key <oem-private-key-file>
     --security-counter <value>
     ${INPUT_FILE} ${OUTPUT_FILE}

Where ``<oem-private-key-file>`` is the path to the OEM private key file (e.g. a ``.pem``
file) matching the public key provisioned into the device, and ``<value>`` is the security
counter assigned during provisioning. Without these additional parameters, images built for a
provisioned device will be rejected by the ROM extended boot.
MCUBoot Bootloader Support
**************************

The ``kit_pse84_eval`` board supports `MCUBoot`_ for bootloader and
over-the-air (OTA) firmware updates. The PSOC™ Edge E84 extended-boot ROM
validates an MCUBoot-compatible image header at the MCUBoot bootloader location
in RRAM before handing off to MCUBoot, which then validates and starts the
application(s).

.. IMPORTANT::
   When using MCUBoot, the ``SW6`` (``BOOT SW``) DIP switch **MUST** be set to
   the **OFF** position. This causes the extended boot to jump to the MCUBoot
   bootloader in RRAM at ``0x22011000``. If the switch is left ON, the extended
   boot will attempt to jump directly to the external flash XIP address,
   bypassing MCUBoot entirely.

The MCUBoot slot partitions are defined in the base memory map
(``kit_pse84_eval_memory_map.dtsi``) alongside the non-MCUBoot XIP partitions,
using dual labels. This means no extra DTS overlay is needed for application
images — only the MCUBoot bootloader itself requires an overlay
(``kit_pse84_eval_mcuboot_bl.overlay``) to select ``boot_partition`` and RRAM
as its flash device.

Flash Layout with MCUBoot
=========================

When MCUBoot is enabled, the flash is partitioned as follows:

.. list-table::
   :header-rows: 1

   * - Partition
     - Location
     - Offset
     - Size
     - Description
   * - ``boot_partition``
     - RRAM
     - ``0x11000``
     - 256 KB
     - MCUBoot bootloader (address ``0x22011000``)
   * - ``slot0_partition``
     - flash0_s
     - ``0x100000``
     - 2.25 MB
     - CM33S primary application (active)
   * - ``slot1_partition``
     - flash0_s
     - ``0x700000``
     - 2.25 MB
     - CM33S secondary / update slot
   * - ``slot2_partition``
     - flash0
     - ``0x340000``
     - 1.75 MB
     - CM33NS primary application (active)
   * - ``slot3_partition``
     - flash0
     - ``0x940000``
     - 1.75 MB
     - CM33NS secondary / update slot
   * - ``slot4_partition``
     - flash0_sahb
     - ``0x500000``
     - 2 MB
     - CM55 primary application (active)
   * - ``slot5_partition``
     - flash0_sahb
     - ``0xB00000``
     - 2 MB
     - CM55 secondary / update slot

The primary slot partitions share the same DTS node as the XIP code partitions
(``m33s_xip`` / ``m33_xip`` / ``m55_xip``), so the same ``zephyr,code-partition``
setting works for both MCUBoot and non-MCUBoot builds. The 0x400-byte image
header is accounted for by ``CONFIG_ROM_START_OFFSET``.

Building Images Independently (Standalone)
==========================================

Each image can be built and flashed individually. Because the slot partitions
are part of the base memory map, no extra DTS overlays are needed for
application images.

The board directory overlays referenced below are located in
``boards/infineon/kit_pse84_eval/`` relative to ``$ZEPHYR_BASE``.

Step 1 — MCUBoot bootloader
----------------------------

The bootloader must be linked at ``boot_partition`` in RRAM. Pass the
bootloader-specific overlay and the MCUBoot configuration file.

The board-level ``kit_pse84_eval_mcuboot.conf`` defaults to
``CONFIG_UPDATEABLE_IMAGE_NUMBER=2`` so that MCUBoot validates two
independent images out of the box (slot0/slot1 for image 0 and
slot2/slot3 for image 1). This matches the TF-M flow (signed SPE
at slot0 + signed NSPE at slot2).

For a **two-image** build (TF-M SPE + NSPE):

.. code-block:: shell

   west build -b kit_pse84_eval/pse846gps2dbzc4a/m33 \
       ./../bootloader/mcuboot/boot/zephyr -d build_mcuboot \
       -- -DEXTRA_DTC_OVERLAY_FILE="$ZEPHYR_BASE/boards/infineon/kit_pse84_eval/kit_pse84_eval_mcuboot_bl.overlay" \
          -DEXTRA_CONF_FILE="$ZEPHYR_BASE/boards/infineon/kit_pse84_eval/kit_pse84_eval_mcuboot.conf"

For a **single-image** setup (CM33S only), override the image count to 1:

.. code-block:: shell

   west build -b kit_pse84_eval/pse846gps2dbzc4a/m33 \
       bootloader/mcuboot/boot/zephyr -d build_mcuboot \
       -- -DEXTRA_DTC_OVERLAY_FILE="$ZEPHYR_BASE/boards/infineon/kit_pse84_eval/kit_pse84_eval_mcuboot_bl.overlay" \
          -DEXTRA_CONF_FILE="$ZEPHYR_BASE/boards/infineon/kit_pse84_eval/kit_pse84_eval_mcuboot.conf" \
          -DCONFIG_UPDATEABLE_IMAGE_NUMBER=1

Step 2 — CM33S application (slot0)
-----------------------------------

No extra DTS overlay is needed. The conf file sets
``CONFIG_BOOTLOADER_MCUBOOT=y``, the matching MCUBoot mode, and unsigned
image generation:

.. code-block:: shell

   west build -b kit_pse84_eval/pse846gps2dbzc4a/m33 \
       <path/to/cm33s/app> -d build_cm33s \
       -- -DEXTRA_CONF_FILE="$ZEPHYR_BASE/boards/infineon/kit_pse84_eval/kit_pse84_eval_slot.conf"

Step 3 — Flashing
-----------------

Each image is flashed independently from its build directory.
``west flash`` automatically selects the correct hex file.

Flash MCUBoot first (required once; re-flash only when updating the
bootloader):

.. code-block:: shell

   west flash -d build_mcuboot

Flash the CM33S application:

.. code-block:: shell

   west flash -d build_cm33s

TF-M with MCUBoot (Multi-Image)
================================

When TF-M is used together with MCUBoot, the TF-M Secure Processing
Environment (SPE) is placed in ``slot0_partition`` (image 0) and the
Non-Secure application (NSPE) in ``slot2_partition`` (image 1). MCUBoot
runs in multi-image mode (``CONFIG_UPDATEABLE_IMAGE_NUMBER=2``, the
board default) and validates the SPE and NSPE independently. TF-M itself
still handles the run-time transition from secure to non-secure state.

Signing is done by the helper in
:file:`soc/infineon/edge/pse84/pse84_tfm_signing.cmake`, which replaces
the default ``cmake/mcuboot.cmake`` flow for PSE84 NS images. It:

- Derives the NSPE slot size from ``slot0_ns_partition`` (the NS view of
  slot2) instead of ``slot0_partition``, so the MCUboot trailer lands at
  the correct offset.
- Signs the NSPE with ``imgtool sign --overwrite-only --align 1`` and the
  same ``ROM_START_OFFSET`` header size used by Zephyr images.
- Merges the signed SPE (produced by TF-M's own build) with the freshly
  signed NSPE into ``tfm_merged.hex`` and points ``west flash`` at it.

Boot flow::

    Extended Boot (ROM) → MCUBoot (RRAM, 0x22011000)
        → validates image 0 (TF-M SPE  @ slot0 / 0x18100000)
        → validates image 1 (NSPE     @ slot2 / 0x18340000)
        → jumps to TF-M SPE
            → TF-M configures TrustZone / MPC / SAU
            → TF-M launches NSPE (@ 0x08340000)

Step 1 — MCUBoot bootloader
---------------------------

Same as the standalone two-image case above — the board default already
validates slot0 and slot2:

.. code-block:: shell

   west build -b kit_pse84_eval/pse846gps2dbzc4a/m33 \
       bootloader/mcuboot/boot/zephyr -d build_mcuboot \
       -- -DEXTRA_DTC_OVERLAY_FILE="$ZEPHYR_BASE/boards/infineon/kit_pse84_eval/kit_pse84_eval_mcuboot_bl.overlay" \
          -DEXTRA_CONF_FILE="$ZEPHYR_BASE/boards/infineon/kit_pse84_eval/kit_pse84_eval_mcuboot.conf"

Step 2 — TF-M NS application (SPE + NSPE)
-----------------------------------------

Building the ``m33/ns`` target with ``CONFIG_BUILD_WITH_TFM=y`` (enabled by
default for the ``/ns`` variant) automatically builds TF-M as the SPE,
signs it and the NSPE independently, and produces ``tfm_merged.hex``
containing both signed images ready to be validated by MCUBoot:

.. code-block:: shell

   west build -b kit_pse84_eval/pse846gps2dbzc4a/m33/ns \
       <path/to/ns/app> -d build_tfm_ns \
       -- -DEXTRA_CONF_FILE="$ZEPHYR_BASE/boards/infineon/kit_pse84_eval/kit_pse84_eval_slot.conf"

Step 3 — Flashing
------------------

Flash MCUBoot (once):

.. code-block:: shell

   west flash -d build_mcuboot

Flash the TF-M + NS application. ``west flash`` automatically uses
``tfm_merged.hex`` which contains the signed SPE (slot0) and signed NSPE
(slot2):

.. code-block:: shell

   west flash -d build_tfm_ns

.. NOTE::
   Because SPE and NSPE are signed and validated as two independent
   MCUBoot images, OTA updates can target either one without re-signing
   the other: a new SPE is staged in slot1 and a new NSPE in slot3.

TF-M Multicore Support
**********************

The PSOC™ Edge E84 supports a TF-M paired-build configuration where the CM33 Non-Secure
application acts as the PSA client local to the secure firmware, and the CM55 Non-Secure
application reaches the same TF-M Secure Processing Environment running on the CM33 through a
mailbox-based relay. This is enabled with the Kconfig option
``CONFIG_PSOC_EDGE_M55_SRF_SUPPORT``, which must be set on **both** images.

.. NOTE::
   The CM55 image in this configuration is **not** built using sysbuild. Both images are built
   as standalone Zephyr applications, and they must be flashed independently. The paired build
   is coordinated through a CMake variable rather than through sysbuild.

Build Order and Dependency
==========================

The CM55 build consumes the PSA manifest headers generated by the CM33-NS TF-M build, so the
CM33-NS image **must be built first**. The CM55 CMake configuration expects the CM33-NS build
output to be available at configure time and will fail with a ``FATAL_ERROR`` if it cannot find
the directory ``<cm33-ns-build>/tfm/generated/interface/include``.

By default the CM55 build looks for the CM33-NS build output at ``${ZEPHYR_BASE}/build``. When
a different output directory is used (e.g. via ``west build -d``), its path must be passed to
the CM55 build through the ``PSE84_CM33_BUILD_DIR`` CMake variable.

Building
========

#. Build the CM33-NS image with mailbox/relay support enabled:

   .. zephyr-app-commands::
      :app: samples/hello_world
      :board: kit_pse84_eval/pse846gps2dbzc4a/m33/ns
      :goals: build
      :gen-args: -DCONFIG_PSOC_EDGE_M55_SRF_SUPPORT=y

   The example below uses an explicit build directory:

   .. code-block:: shell

      west build -b kit_pse84_eval/pse846gps2dbzc4a/m33/ns \
                 -d build_multicore_33 samples/hello_world \
                 -- -DCONFIG_PSOC_EDGE_M55_SRF_SUPPORT=y

#. Build the CM55-NS image, pointing it at the CM33-NS build directory from the previous step
   via ``PSE84_CM33_BUILD_DIR``:

   .. code-block:: shell

      west build -b kit_pse84_eval/pse846gps2dbzc4a/m55 \
                 -d build_multicore_55 samples/basic/blinky \
                 -- -DCONFIG_PSOC_EDGE_M55_SRF_SUPPORT=y \
                    -DPSE84_CM33_BUILD_DIR=$(pwd)/build_multicore_33

   If the CM33-NS image was built with the default ``build`` directory, the
   ``-DPSE84_CM33_BUILD_DIR=...`` argument may be omitted.

Flashing
========

The two images are independent flash artifacts and must be programmed separately. Flash the
CM55 image first as the CM33-NS image will try to boot the CM55 image on reset and will fault
if it cannot find a valid image to jump to.

.. code-block:: shell

   west flash -d build_multicore_55
   west flash -d build_multicore_33

References
**********

- `PSOC™ Edge E84 Arm® Cortex® Multicore SoC Website`_

.. _PSOC™ Edge E84 Arm® Cortex® Multicore SoC Website:
    https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm/psoc-edge-e84#Overview

.. _PSOC™ Edge E84 Evaluation Kit Website:
    https://www.infineon.com/evaluation-board/KIT-PSE84-EVAL

.. _kit_pse84_eval User Manual Website:
    https://www.infineon.com/assets/row/public/documents/30/44/infineon-kit-pse84-eval-qsg-usermanual-en.pdf

.. _PSOC™ Edge Security Getting Started Application Note:
    https://www.infineon.com/assets/row/public/documents/30/42/infineon-an237849-getting-started-psoc-edge-security-applicationnotes-en.pdf

.. _ModusToolbox™:
    https://softwaretools.infineon.com/tools/com.ifx.tb.tool.modustoolboxsetup

.. _ModusToolbox™ software installation guide:
    https://www.Infineon.com/ModusToolboxInstallguide

.. _KitProg3:
    https://github.com/Infineon/KitProg3

.. _MCUBoot:
    https://docs.mcuboot.com/
