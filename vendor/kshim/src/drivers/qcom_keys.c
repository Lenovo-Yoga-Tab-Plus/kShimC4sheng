#if !defined(KSHIM_HOST_TEST)
#include <config.h>
#endif
#include <qcom_key_platform.h>
#include <qcom_keys.h>
#include <stdint.h>
#include <string.h>
#include <Library/spmi.h>
#include <Library/pmic_gpio.h>
#include <Library/gpio.h>

#if defined(__aarch64__)
#include <timer.h>
#endif
__attribute__((weak)) void kshim_qcom_spmi_delay_us(uint32_t Microseconds)
{
#if defined(__aarch64__)
  (void)delay(Microseconds);
#else
  (void)Microseconds;
#endif
}

#if CONFIG_QCOM_SPMI
#define SPMI_PORTS 4
struct spmi_port { SpmiDeviceContext *spmi; int fault; uintptr_t command; };
static struct spmi_port ports[SPMI_PORTS];
static unsigned port_count;
void kshim_qcom_spmi_bind(SpmiDeviceContext **contexts, unsigned count)
{
    memset(ports, 0, sizeof(ports));
    port_count = count <= SPMI_PORTS ? count : 0;
    for (unsigned i = 0; i < port_count; ++i) ports[i].spmi = contexts[i];
}

static int KshimSpmiContains(const SpmiMemoryRegion *Region,
                             uintptr_t Address, size_t Bytes)
{
  return Region->BaseAddress != 0 && Region->Size >= Bytes &&
         Region->BaseAddress <= UINTPTR_MAX - (Region->Size - 1U) &&
         Address >= Region->BaseAddress &&
         Address - Region->BaseAddress <= Region->Size - Bytes &&
         (Address & 3U) == 0;
}

static int KshimSpmiAddressValid(struct spmi_port *port, uintptr_t Address)
{
  for (unsigned Index = 0; Index < SPMI_MEMORY_REGION_TYPE_MAX; Index++)
    if (KshimSpmiContains(&port->spmi->Regions[Index], Address, sizeof(uint32_t)))
      return 1;
  port->fault = 1;
  return 0;
}

static void KshimSpmiBarrier(void)
{
#if defined(__aarch64__)
  __asm__ volatile("dsb sy" ::: "memory");
#else
  __asm__ volatile("" ::: "memory");
#endif
}

static UINT32 KshimSpmiRead(void *Context, UINTN Address)
{
  struct spmi_port *port = Context;
  if (!KshimSpmiAddressValid(port, Address))
    return 0;
  uint32_t Value = *(volatile uint32_t *)Address;
  KshimSpmiBarrier();
  return Value;
}

static void KshimSpmiWrite(void *Context, UINTN Address, UINT32 Value)
{
  struct spmi_port *port = Context;
  /* The key adapter only submits a read command. No PMIC configuration or
   * IRQ ownership writes are exposed through this polled integration. */
  if (!KshimSpmiAddressValid(port, Address) || Address != port->command) {
    port->fault = 1;
    return;
  }
  KshimSpmiBarrier();
  *(volatile uint32_t *)Address = Value;
  KshimSpmiBarrier();
}

static void KshimSpmiStall(void *Context, UINT32 Microseconds)
{
  (void)Context;
  kshim_qcom_spmi_delay_us(Microseconds);
}

static int KshimSpmiInit(struct spmi_port *port)
{
  SpmiDeviceContext *ctx = port->spmi;
  if (!ctx) return -1;
  if (ctx->Initialized) return 0;
  port->fault = 0;
  ctx->Io = (SpmiIoOps){KshimSpmiRead, KshimSpmiWrite, KshimSpmiStall, port};
  if (CR_ERROR(SpmiLibInit(&ctx)) || port->fault) {
    ctx->Initialized = FALSE;
    return -1;
  }
  return 0;
}

/* CrDK supplies Linux-compatible PMIC Arbiter v1/v2/v3/v5/v7/v8/v8.5
 * transactions. The compatibility API returns the single PMIC status byte. */
static int spmi_read(struct spmi_port *port, uint8_t Sid, uint32_t Address, uint32_t *Value)
{
  uint8_t Byte;
  uint32_t Offset;
  SpmiMemoryRegion *ReadRegion;
  if (Value == NULL || Sid > 15U || Address > UINT16_MAX ||
      KshimSpmiInit(port) != 0)
    return -1;
  port->fault = 0;
  Offset = port->spmi->PmicArb.Ops->GetChannelOffset(
      port->spmi, Sid, (uint16_t)Address, SPMI_ARB_CHANNEL_OBS);
  ReadRegion = &port->spmi->Regions[port->spmi->Version == SPMI_ARB_VERSION_1
                                ? SPMI_MEMORY_REGION_TYPE_CORE
                                : SPMI_MEMORY_REGION_TYPE_OBSERVER];
  if (Offset == UINT32_MAX || port->fault ||
      ReadRegion->BaseAddress > UINTPTR_MAX - Offset ||
      !KshimSpmiContains(ReadRegion, ReadRegion->BaseAddress + Offset,
                        SPMI_PMIC_ARB_RDATA0 + sizeof(uint32_t)))
    return -1;
  port->command = ReadRegion->BaseAddress + Offset;
  CR_STATUS Status = SpmiRead(port->spmi, Sid, (uint16_t)Address, &Byte, 1);
  port->command = 0;
  if (CR_ERROR(Status) || port->fault)
    return -1;
  *Value = Byte;
  return 0;
}

static CR_STATUS pmic_read(void *context, UINT8 sid, UINT16 address, UINT8 *buffer, UINTN size)
{
  uint32_t value;
  if (!buffer || size != 1 || spmi_read(context, sid, address, &value)) return CR_DEVICE_ERROR;
  *buffer = value;
  return CR_SUCCESS;
}

__attribute__((weak)) int kshim_qcom_spmi_read32(void *context, uint8_t sid,
                                               uint32_t address, uint32_t *value)
{
  (void)context;
  return port_count ? spmi_read(&ports[0], sid, address, value) : -1;
}

#if defined(__aarch64__) && !defined(KSHIM_HOST_TEST)
/* TB520FU volume keys are SPMI observer reads: PON 0x1310 (power bit 7,
 * volume-down bit 6) and pm8550 gpio6 0x8D10 (volume-up, active low).
 * The arbiter init used above can fail closed on this v7 map, which leaves
 * the menu up and the keys dead. This is the same observer sequence the
 * boot selector already uses on this tablet. */
static int v7_mmio_read(uint64_t address, uint32_t *value)
{
  if ((address & 3U) != 0U) return -1;
  *value = *(volatile uint32_t *)(uintptr_t)address;
  __asm__ volatile("dsb sy" ::: "memory");
  return 0;
}

static int v7_mmio_write(uint64_t address, uint32_t value)
{
  if ((address & 3U) != 0U) return -1;
  __asm__ volatile("dsb sy" ::: "memory");
  *(volatile uint32_t *)(uintptr_t)address = value;
  __asm__ volatile("dsb sy" ::: "memory");
  return 0;
}

static uint64_t v7_ticks(void)
{
  uint64_t ticks;
  __asm__ volatile("isb\n mrs %0, cntvct_el0" : "=r"(ticks));
  return ticks;
}

static uint64_t v7_freq(void)
{
  uint64_t frequency;
  __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(frequency));
  return frequency;
}

static int v7_apid(uint64_t core, uint64_t cfg, uint8_t ee, uint16_t ppid, uint16_t *apid)
{
  struct slot { uint64_t core; uint16_t ppid, apid; uint8_t ee, state; };
  static struct slot cache[4];
  uint32_t version, count;
  uint16_t hit = 0;
  int found = 0;
  unsigned i;

  for (i = 0; i < 4U; i++) {
    if (cache[i].state && cache[i].core == core && cache[i].ppid == ppid && cache[i].ee == ee) {
      if (cache[i].state != 1U) return -1;
      *apid = cache[i].apid;
      return 0;
    }
  }
  if (core < 0x1000U || cfg < 0x1000U || v7_mmio_read(core, &version) ||
      v7_mmio_read(core + 4U, &count))
    return -1;
  if (version < 0x70000000U || version >= 0x80000000U) return -2;
  count &= 0x7ffU;
  if (count > 1024U) count = 1024U;
  for (i = 0; i < count; i++) {
    uint32_t map, owner;
    if (v7_mmio_read(core + 0x2000U + 4U * i, &map)) return -1;
    if (!map || ((map >> 8) & 0xfffU) != ppid) continue;
    if (v7_mmio_read(cfg + 4U * i, &owner)) return -1;
    if ((owner & 7U) == ee) { hit = (uint16_t)i; found = 1; }
  }
  for (i = 0; i < 4U; i++) {
    if (cache[i].state) continue;
    cache[i] = (struct slot){core, ppid, hit, ee, found ? 1U : 2U};
    break;
  }
  if (!found) return -1;
  *apid = hit;
  return 0;
}

static int v7_read_byte(uint64_t obs, uint16_t apid, uint16_t reg, uint32_t *value)
{
  uint64_t channel, frequency, started;
  uint32_t command, state, data;
  unsigned spin;

  if (obs < 0x1000U || apid >= 1024U) return -1;
  channel = obs + 0x20ULL * apid;
  command = (1U << 27) | ((reg & 0xffU) << 4);
  frequency = v7_freq();
  started = v7_ticks();
  if (frequency < 1000000ULL || frequency > 2000000000ULL || v7_mmio_write(channel, command))
    return -1;
  for (spin = 0; spin < 100000U; spin++) {
    if (v7_mmio_read(channel + 8U, &state)) return -1;
    if ((state & 1U) == 0U) {
      if (v7_ticks() - started >= frequency / 1000ULL) return -1;
      continue;
    }
    if ((state & 0xeU) || v7_mmio_read(channel + 0x18U, &data)) return -1;
    *value = (uint8_t)data;
    return 0;
  }
  return -1;
}

static int kshim_spmi_v7_key_read(uint64_t token, uint32_t *value)
{
  uint8_t sid = QCOM_KEY_SPMI_TOKEN_SID(token);
  uint32_t reg = QCOM_KEY_SPMI_TOKEN_ADDRESS(token);
  uint16_t ppid, apid;
  unsigned index;
  int status = -1;

  if (reg > UINT16_MAX) return -1;
  ppid = (uint16_t)(((uint16_t)sid << 8) | (reg >> 8));
  index = (unsigned)QCOM_KEY_SPMI_TOKEN_CONTROLLER(token);
  if (index < port_count && ports[index].spmi) {
    SpmiDeviceContext *ctx = ports[index].spmi;
    uint64_t core = ctx->Regions[SPMI_MEMORY_REGION_TYPE_CORE].BaseAddress;
    uint64_t cfg = ctx->Regions[SPMI_MEMORY_REGION_TYPE_CONFIG].BaseAddress;
    uint64_t obs = ctx->Regions[SPMI_MEMORY_REGION_TYPE_OBSERVER].BaseAddress;
    status = v7_apid(core, cfg, ctx->ActiveEE, ppid, &apid);
    if (!status) status = v7_read_byte(obs, apid, (uint16_t)reg, value);
    if (!status) return 0;
    if (core == 0x0c400000ULL && cfg == 0x0c42d000ULL && obs == 0x0c440000ULL)
      return -1;
  }
  status = v7_apid(0x0c400000ULL, 0x0c42d000ULL, 0, ppid, &apid);
  if (!status) status = v7_read_byte(0x0c440000ULL, apid, (uint16_t)reg, value);
  return status ? -1 : 0;
}
#endif

#endif

int kshim_qcom_key_read32_status(
    void *Context, uint64_t Address, uint32_t *Value)
{
  (void)Context;
  if (Value == NULL)
    return -1;
  if (QCOM_KEY_IS_SPMI_TOKEN(Address)) {
#if CONFIG_QCOM_SPMI
#if defined(__aarch64__) && !defined(KSHIM_HOST_TEST)
    if (kshim_spmi_v7_key_read(Address, Value) == 0) return 0;
#endif
    unsigned index = QCOM_KEY_SPMI_TOKEN_CONTROLLER(Address);
    if (index >= port_count || !ports[index].spmi ||
        ports[index].spmi->BusId != QCOM_KEY_SPMI_TOKEN_BUS(Address)) return -1;
    if (Address & QCOM_KEY_PMIC_GPIO_FLAG) {
      uint32_t offset = QCOM_KEY_SPMI_TOKEN_ADDRESS(Address);
      if (offset > UINT16_MAX || (offset & 0xff) != 0x10) return -1;
      PmicGpioBusOps bus = {.Read = pmic_read, .Context = &ports[index]};
      uint8_t value;
      if (CR_ERROR(PmicGpioReadStatus(&bus, QCOM_KEY_SPMI_TOKEN_SID(Address), offset - 0x10, &value))) return -1;
      *Value = value;
      return 0;
    }
    return spmi_read(&ports[index], QCOM_KEY_SPMI_TOKEN_SID(Address),
                     QCOM_KEY_SPMI_TOKEN_ADDRESS(Address), Value);
#else
    return -1;
#endif
  }
  if (Address == 0U)
    return -1;
  if ((Address & 3) || Address < 4) return -1;
  static GpioDeviceContext gpio;
  gpio.PinCount = 1;
  gpio.RegCfg[0] = (GpioConfigRegInfo){.IoRegOffset = 4, .InputMsk = 1};
  gpio.GpioPins[0].PinAddress = Address - 4;
  gpio.GpioPins[0].pRegCfg = &gpio.RegCfg[0];
  *Value = GpioReadInputVal(&gpio, 0);
  return 0;
}

uint32_t kshim_qcom_key_read32(void *Context, uint64_t Address)
{
  uint32_t Value;
  return kshim_qcom_key_read32_status(Context, Address, &Value) == 0
             ? Value : UINT32_MAX;
}
