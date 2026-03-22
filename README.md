# Balança inteligente GLP com ESP32

Projeto base em PlatformIO/Arduino para montar uma balança de GLP com:

- ESP32;
- célula de carga de 50 kg com HX711;
- termistor NTC 10 k;
- captive portal para provisionamento inicial do Wi‑Fi;
- atualização OTA via ArduinoOTA;
- interface web local com `ESPAsyncWebServer` para tara e parametrização;
- telemetria MQTT com métricas de massa e volume estimado de GLP.

## Hardware sugerido

- ESP32 DevKit;
- módulo HX711;
- célula de carga 50 kg;
- termistor 10 k Beta 3950 com resistor série de 10 k;
- fonte estável e caixa IP adequada ao ambiente.

### Ligações padrão no firmware

| Função | GPIO ESP32 |
|---|---:|
| HX711 DOUT | 19 |
| HX711 SCK | 18 |
| Termistor ADC | 34 |

## Premissas do cálculo

O firmware foi inicializado com parâmetros compatíveis com o cenário descrito:

- massa total do conjunto cheio: **45 kg**;
- tara do recipiente: **32 kg**;
- GLP útil: **13 kg**.

A massa líquida de GLP é calculada como:

```text
massa_glp = massa_total - tara
```

O percentual de carga usa `fullGlpKg` como referência. O volume estimado usa densidade padrão configurável (`0,54 kg/L` por padrão).

## Configuração inicial

Na primeira energização, ou após reset das credenciais, o dispositivo sobe um AP:

- SSID: `GLP-Scale-Setup`
- Senha: `glp12345`

Ao conectar no AP, o captive portal assíncrono permite configurar o Wi‑Fi da rede local. Depois disso, os parâmetros operacionais e MQTT podem ser ajustados pela interface web principal.

## Interface web local

Depois de conectado à rede local, acesse:

```text
http://<ip-do-esp32>/
```

Funções disponíveis:

- visualizar massa total, massa líquida de GLP, nível estimado, volume estimado e temperatura;
- gravar a tara atual do recipiente;
- zerar a plataforma vazia para atualizar o offset do HX711;
- editar fator de calibração do HX711;
- ajustar tara, capacidade útil, densidade do GLP e parâmetros do termistor;
- configurar broker MQTT, porta, credenciais, tópico base e uso de TLS;
- reiniciar o equipamento.

> Observação: nesta revisão o servidor foi migrado para `ESPAsyncWebServer`. Como essa pilha opera em HTTP assíncrono, o acesso local ficou em HTTP. Se HTTPS local for obrigatório, a recomendação prática é colocar um gateway/reverse proxy TLS na rede ou migrar para uma pilha de servidor compatível com TLS embarcado.

## OTA

A atualização remota usa `ArduinoOTA`. Depois que o ESP32 estiver na rede, ele poderá aparecer no Arduino IDE/PlatformIO pelo hostname configurado (`deviceId`).

## MQTT publicado

O payload é enviado em:

```text
<topico-base>/<deviceId>/state
```

Exemplo de JSON:

```json
{
  "deviceId": "balanca-glp-01",
  "ip": "192.168.1.50",
  "rssi": -58,
  "grossKg": 44.821,
  "tareKg": 32.000,
  "netGlpKg": 12.821,
  "levelPct": 98.62,
  "estimatedLiters": 23.74,
  "temperatureC": 28.31,
  "mqttConnected": true,
  "freeHeap": 187320
}
```

Isso fornece dados usuais para gestão operacional do GLP, como:

- massa bruta;
- tara configurada;
- GLP líquido disponível;
- percentual estimado de carga;
- volume estimado em litros;
- temperatura local;
- qualidade do sinal Wi‑Fi;
- disponibilidade do nó.

## Ajustes recomendados antes de produção

1. **Zerar a plataforma vazia** pela interface para armazenar o `hxOffset` inicial do HX711.
2. **Calibrar o HX711** com massa padrão conhecida e ajustar `scaleFactor`.
3. **Validar a curva do termistor** conforme o componente real utilizado.
4. **Definir autenticação forte** no broker MQTT e, se necessário, substituir `setInsecure()` por validação CA.
5. **Proteger a mecânica** da balança para minimizar erro por vibração, vento e desalinhamento do butijão.
6. **Se HTTPS local for mandatório**, usar um gateway TLS na rede industrial/comercial ou trocar a pilha de servidor.

## Build

```bash
pio run
```

## Upload

```bash
pio run -t upload
```

## Próximos passos sugeridos

- adicionar calibração guiada na interface web;
- publicar alarmes MQTT por limiar mínimo de GLP;
- armazenar histórico local ou integrar com Home Assistant / Node-RED / SCADA;
- incluir compensação térmica e filtragem digital mais robusta.
