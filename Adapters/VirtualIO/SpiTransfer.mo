within ModelicaCPSLibrary.Adapters.VirtualIO;
block SpiTransfer "Draft boardrunner byte-request to ideal SPI-signal boundary"
  extends Logical.Communications.SPI.ByteMaster;
  annotation(Documentation(info="<html><p>One instance represents one selected slave endpoint. requestId is an event identifier, not a hardware SPI pin. Set CS before submitting, hold it through completion, and wait for completedId before the next request. The adapter synthesizes an ideal clock from clockFrequency; it does not observe boardrunner pin timing. See docs/spi_interface.md for the proposed host contract and integration limits.</p></html>"));
end SpiTransfer;
